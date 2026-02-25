#include "Optimize.h"

#include <ceres/ceres.h>
#include <ceres/cubic_interpolation.h>
#include <chrono>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>


///////////for 3d/////////////
#include <omp.h>
//////////end

namespace OptimizeCeres {
	// 第一部分：代价函数的计算模型，重载（）符号，仿函数的小技巧 
	struct CostFunctor
	{
		CostFunctor(cv::Point3f _XYZ, cv::Point _xy)
		{
			XYZ = _XYZ;
			xy = _xy;
		}
		// 残差的计算
		template <typename T>
		bool operator() (const T* const camPose, T* residual) const     // 残差
		{
			vector<T> cc = { camPose[0], camPose[1], camPose[2], camPose[3], camPose[4], camPose[5] };
			T pp_xy[2];
			T ppt_3d[3] = { T(XYZ.x), T(XYZ.y),T(XYZ.z) };

			//if (convertPoint3dTo2d(cc, ppt_3d, pp_xy))
			if (convertPoint3dTo2d(cc, ppt_3d, pp_xy, false))
			{
				residual[0] = T(xy.x) - T(pp_xy[0]);
				residual[1] = T(xy.y) - T(pp_xy[1]);
			}
			else
			{
				residual[0] = T(CalibSpace::IMG_WIDTH);
				residual[1] = T(CalibSpace::IMG_HEIGHT);
			}

			return true; //千万不要写成return 0,要写成return true
		}
	private:
		cv::Point xy;
		cv::Point3f XYZ;
	};



	template<typename T>
	T sigmoid(T x)
	{
		if (x > 0)
			return T(1.0) / (T(1.0) + exp(-x));
		else
			return  exp(x) / (T(1.0) + exp(x));
	}

	template<typename T>
	T tanh(T x)
	{
		//return (exp(x) - exp(-x)) / (exp(x) + exp(-x));
		if (x > 0)
			return (1 - exp(-T(2.0) * x)) / (1 + exp(-T(2.0) * x));
		else
			return (exp(T(2.0) * x) - 1) / (1 + exp(T(2.0) * x));
	}

	struct CostFunctorIOU
	{
		CostFunctorIOU(const ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>& _interpolator,
			const cv::Point3f& _XYZ, int _gray) :
			interpolator(_interpolator), XYZ(_XYZ), gray_ref(_gray) {}

		// 残差的计算
		template <typename T>
		bool operator() (const T* const camPose,
			T* residual)const      // 残差
		{
			vector<T> cc = { camPose[0], camPose[1], camPose[2], camPose[3], camPose[4], camPose[5] };
			int non_zero_eval = 0;

			T gray_img;
			T pp_xy[2];
			T pXYZ[3] = { T(XYZ.x), T(XYZ.y),T(XYZ.z) };

			if (convertPoint3dTo2d(cc, pXYZ, pp_xy, false))
			{
				interpolator.Evaluate(pp_xy[1], pp_xy[0], &gray_img);
			}
			else
			{
				gray_img = T(1E-3);
			}
			// Set residual
			//cross entropy
			if (gray_img < T(1E-2))
			{
				gray_img = T(1E-2);
			}
			if (gray_img > T(1 - 1E-2))
			{
				gray_img = T(1 - 1E-2);
			}
			// cross entropy
			residual[0] = T(gray_ref) * log2(gray_img) + T(1 - gray_ref) * log2(T(1) - gray_img);
			if (CalibSpace::activate_flg)
			{
				//激活
				residual[0] = tanh(residual[0]);
			}
			else
			{
				//不激活
			}

			return true;
		}
	private:
		const ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>>& interpolator;
		cv::Point3f XYZ;
		int gray_ref;
	};


	void  optimize(const vector<CalibSpace::Point3d2d>& ref_vec, vector<double>& cam)
	{
		if (cam.size() != 6)
		{
			return;
		}

		double camPose[6];
		for (int i = 0; i < 6; i++)
		{
			camPose[i] = cam[i];
		}
		//第二部分：构建寻优问题
		ceres::Problem problem;
		try
		{
			for (int i = 0; i < ref_vec.size(); ++i)
			{
				//使用自动求导，将之前的代价函数结构体传入，第一个是输出维度，即残差的维度，第二个是输入维度，即待寻优参数x的维度
				const auto& ref = ref_vec[i];
				ceres::CostFunction* costfunction = new ceres::AutoDiffCostFunction<CostFunctor, 2, 6>(new CostFunctor(ref.p3d, ref.p2d));

				//将残差方程和观测值加入到problem,nullptr表示核函数为无，
				problem.AddResidualBlock(costfunction, new ceres::SoftLOneLoss(1.0), camPose);
			}
		}
		catch (...)
		{
			cout << "costFunction error" << endl;
		}

		//添加边界约束
				//x
		for (int i = 0; i < 6; i++)
		{
			double min_b = 0;
			double max_b = 0;
			if (i < 3)
			{
				min_b = camPose[i] - 1;
				max_b = camPose[i] + 1;
			}
			else
			{
				min_b = camPose[i] - 0.08;
				max_b = camPose[i] + 0.08;
			}
			problem.SetParameterLowerBound(camPose, i, min_b);
			problem.SetParameterUpperBound(camPose, i, max_b);
		}

		// 第三部分：配置求解器
		ceres::Solver::Options options;
		// 这里有很多配置项可以填
		options.linear_solver_type = ceres::DENSE_QR;  // 配置增量方程的解法

		ceres::Solver::Summary summary;  // 优化信息
		chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
		ceres::Solve(options, &problem, &summary);  // 开始优化
		chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
		chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

		for (int i = 0; i < 6; i++)
		{
			cam[i] = camPose[i];
		}
	}

	void optimizeCrossEntropy(const map<int, Point3f_VEC>& xyz_vec_map, const cv::Mat& blur_mat, vector<double>& cam)
	{
		double camPose[6];
		for (int i = 0; i < 6; i++)
		{
			camPose[i] = cam[i];
		}

		ceres::Grid2D<float, 1> image_array((float*)blur_mat.data, 0, blur_mat.size().height, 0, blur_mat.size().width);
		ceres::BiCubicInterpolator<ceres::Grid2D<float, 1>> base_interpolator(image_array);

		//第二部分：构建寻优问题

		ceres::Problem problem;

		vector<cv::Point> good_markers;
		vector<cv::Point> bad_markers;

		auto itr_type = xyz_vec_map.begin();
		for (; itr_type != xyz_vec_map.end(); itr_type++)
		{
			double type = itr_type->first;
			const auto& xyz_vec = itr_type->second;

			for (int i = 0; i < xyz_vec.size(); ++i)
			{
				const auto& xyz = xyz_vec[i];

#if 1
				vector<double> extend_x_vec = { -0.5, 0.5 };
				if (xyz.z < 20)
				{
					for (int i = 0; i < extend_x_vec.size(); i++)
					{
						auto extend_xyz = xyz;
						extend_xyz.x += extend_x_vec[i];

						//double grey = extend_grey_vec[i];
						double grey = 0.0;
						ceres::CostFunction* costfunction =
							new ceres::AutoDiffCostFunction<CostFunctorIOU, 1, 6>(
								new CostFunctorIOU(base_interpolator, extend_xyz, grey));

						problem.AddResidualBlock(costfunction,
							new ceres::SoftLOneLoss(1.0),
							camPose);

						/*/// /绘制用
						double p3[3] = { extend_xyz.x, extend_xyz.y, extend_xyz.z };
						double p2[2];
						if (convertPoint3dTo2d(cam, p3, p2, false))
						{
							bad_markers.push_back(cv::Point(p2[0], p2[1]));
						}*/
					}
				}
				ceres::CostFunction* costfunction =
					new ceres::AutoDiffCostFunction<CostFunctorIOU, 1, 6>(
						new CostFunctorIOU(base_interpolator, xyz, 1.0));

				problem.AddResidualBlock(costfunction,
					new ceres::SoftLOneLoss(1.0),
					camPose);

				/*/// /绘制用
				double p3[3] = { xyz.x, xyz.y, xyz.z };
				double p2[2];
				if (convertPoint3dTo2d(cam, p3, p2, false))
				{
					good_markers.push_back(cv::Point(p2[0], p2[1]));
				}*/
#else
				ceres::CostFunction* costfunction =
					new ceres::AutoDiffCostFunction<CostFunctorIOU, 1, 6>(
						new CostFunctorIOU(base_interpolator, xyz, 1.0));

				problem.AddResidualBlock(costfunction,
					new ceres::SoftLOneLoss(1.0),
					camPose);
#endif
			}
		}

		//////绘制IOU示意图用
		//cv::Mat mat = cv::imread("0_dt.jpg");
		//for_each(good_markers.begin(), good_markers.end(), [&mat](const auto& marker) {
		//	cv::circle(mat, marker, 1, cv::Scalar(0, 255,0), 2);
		//	});
		//for_each(bad_markers.begin(), bad_markers.end(), [&mat](const auto& marker) {
		//	cv::circle(mat, marker, 1, cv::Scalar(0, 0, 255), 2);
		//	});
		//cv::imwrite("0_dt_hd.jpg", mat);

		//添加边界约束
			//x
		for (int i = 0; i < 6; i++)
		{
			double min_b = 0;
			double max_b = 0;
			if (i < 3)
			{
				min_b = camPose[i] - 1;
				max_b = camPose[i] + 1;
			}
			else
			{
				min_b = camPose[i] - 0.08;
				max_b = camPose[i] + 0.08;
			}
			problem.SetParameterLowerBound(camPose, i, min_b);
			problem.SetParameterUpperBound(camPose, i, max_b);
		}

		// 第三部分：配置求解器
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::DENSE_QR;  // 配置增量方程的解法
		options.minimizer_progress_to_stdout = false;   // 输出到cout

		ceres::Solver::Summary summary;  // 优化信息
		chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
		ceres::Solve(options, &problem, &summary);  // 开始优化
		chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
		chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
		LOG(INFO) << "solve time cost = " << time_used.count() << " seconds. ";

		for (int i = 0; i < 6; i++)
		{
			cam[i] = camPose[i];
		}
	}

	double evaluate(vector<CalibSpace::Point3d2d>& ref_vec, const vector<double>& camPose)
	{
		int d = 0;
		double ref_total = 0;
		for (int i = 0; i < ref_vec.size(); ++i)
		{
			auto& ref = ref_vec[i];
			double xx = -1;
			double yy = -1;

			//double pc[3];
			//convertPointByCeres(camPose, X[i], Y[i], Z[i], pc);
			double p3[3] = { ref.p3d.x, ref.p3d.y, ref.p3d.z };
			double p2[2] = { -1 };
			if (convertPoint3dTo2d(camPose, p3, p2))
			{
				double res_x = p2[0] - ref.p2d.x;
				double res_y = p2[1] - ref.p2d.y;
				ref.res = sqrt(res_x * res_x + res_y * res_y);
				ref_total += ref.res;
				d++;
			}
			else
			{
				ref.res = -1;
			}
		}

		if (d != 0)
		{
			ref_total /= d;
		}
		else
		{
			ref_total = 1000;
		}

		return ref_total;
	}
}