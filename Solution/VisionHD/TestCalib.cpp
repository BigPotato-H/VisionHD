// Calib.cpp : 定义控制台应用程序的入口点。
// for std
#include <iostream>
#include <io.h>
#include <direct.h>
// for opencv
#include "TestCalib.h"
#include "CenterLine.h"
#include "HNMath/GeometricAlgorithm2.h"

#include <iostream>
#include<fstream>
#include <glog/logging.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/base.hpp>


#include "DataIO.h"
#include "CCalib.h"

#include "Optimize.h"

using namespace std;
using namespace cv;

#if 0
double calcD(int x, int cycle)
{
	x %= cycle;

	//int c = 2;
	float c = 2.7;
	double y = 10 * (x - c) * (x - c) + 20;

	//int c = 1;
	//double y = 10 * (x - c) * (x - c) + 10;

	return y;
}
#else
double calcD(int x, int cycle)
{
	x %= cycle;
	float c = 2.7;
	double y = 10 * (x - c) * (x - c) + 20;
	y = 20 * (cycle - x);
	return y;
}
#endif
bool searchNeareatPoint(const vector<cv::Point>& d2_pts, cv::flann::Index& kdtree,
	const cv::Point& d2_pt, int& pixel_idx, double& nearest_dist, double D)
{
	unsigned knn = 1;//用于设置返回邻近点的个数
	vector<float> vecQuery(2);//存放 查询点 的容器（本例都是vector类型）
	cv::flann::SearchParams params(-1);//设置knnSearch搜索参数

	/**KD树knn查询**/
	vecQuery[0] = d2_pt.x; //查询点x坐标
	vecQuery[1] = d2_pt.y; //查询点y坐标

	vector<int> vecIndex/*(knn)*/;//存放返回的点索引
	vector<float> vecDist/*(knn)*/;//存放距离

	kdtree.knnSearch(vecQuery, vecIndex, vecDist, knn, params);
	pixel_idx = vecIndex[0];
	if (pixel_idx == 0 && vecDist[0] < 0.01)
	{
//		LOG(INFO) << "knn can't search nearest point....";
		return false;
	}
	//cv::Point search_pt = d2_pts[pixel_idx];

	nearest_dist = sqrt(vecDist[0]);
	if (nearest_dist > D)
	{
		return false;
	}
	return true;
}

Calib::Calib()
{

}

Calib::~Calib()
{
}


void Calib::calcBlurImage(const vector<vector<cv::Point>>& line_vec, cv::Mat& blur_mat)
{
	blur_mat = cv::Mat(CalibSpace::IMG_HEIGHT, CalibSpace::IMG_WIDTH, CV_8UC1, cv::Scalar(0));
	cv::drawContours(blur_mat, line_vec, -1, cv::Scalar(255));
	cv::GaussianBlur(blur_mat, blur_mat, cv::Size(7, 7), 0, 0);
	cv::threshold(blur_mat, blur_mat, 1, 1, cv::THRESH_BINARY);
}

void Calib::calcBlurImage(const map<int, LINE_VEC>& line_vec_map, cv::Mat& blur_mat)
{
	blur_mat = cv::Mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC1, cv::Scalar(0));
	//当前是逆透视投影，固定尺寸800*1280
	//blur_mat = cv::Mat(1280,800, CV_8UC1, cv::Scalar(0));
	auto itr = line_vec_map.begin();
	for (; itr != line_vec_map.end(); itr++)
	{
		const auto& type = itr->first;
		const auto& line_vec = itr->second;
		cv::polylines(blur_mat, line_vec, false, cv::Scalar(255), 20);
	}
}

void Calib::calcDistanceTransformImage(const map<int, LINE_VEC>& line_vec_map, 
cv::Mat& blur_mat)
{
	cv::Mat mat = cv::Mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC1, cv::Scalar(0));

	auto itr = line_vec_map.begin();
	for (; itr != line_vec_map.end(); itr++)
	{
		const auto& type = itr->first;
		const auto& line_vec = itr->second;
	//	cv::polylines(mat, line_vec, false, cv::Scalar(255), 100);
		cv::polylines(mat, line_vec, false, cv::Scalar(255), 30);
	}

	cv::distanceTransform(mat, blur_mat, DIST_L1, 3);
	cv::normalize(blur_mat, blur_mat, 0, 255, cv::NORM_MINMAX);

}

void recoverContourSize(cv::Size& img_size,LINE_VEC& line_vec)
{
	cv::Size recover_size = cv::Size(CalibSpace::image_rect.width, CalibSpace::image_rect.height);
	for_each(line_vec.begin(), line_vec.end(), [&](auto& line)
	{
		for_each(line.begin(), line.end(), [&](auto& pt) {
			pt.x = pt.x * 1.0 / img_size.width * recover_size.width;
			pt.y = pt.y * 1.0 / img_size.height * recover_size.height;
		});
	});
}

void recoverImageSize(cv::Mat& img)
{
	if (CalibSpace::camera_type == CAMERA_MSS_PANO)
	{
		int cut_size = 2048;
		cv::resize(img, img, cv::Size(cut_size, cut_size), INTER_NEAREST);
	}
	else if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
	{
		cv::resize(img, img, cv::Size(CalibSpace::IMG_WIDTH, CalibSpace::IMG_HEIGHT), INTER_NEAREST);
	}
	else
	{

	}

}

void Calib::extractImageDeepLearningMultiLines(const string& folder_path,
	const string& img_na,
	const set<ObjectClassification>& oc_set, 
	map<int, LINE_VEC>& lines_map,
	int mask_type)
{
	string ld_mask_path = "";
	//road mask
	if (mask_type == 0)
	{
		ld_mask_path = folder_path + "Dlink/" + img_na + ".png";
	}
	else
	{
		return;
	}
	if (_access(ld_mask_path.c_str(), 0) != 0)
	{
		return;
	}

	cv::Mat mask = cv::imread(ld_mask_path, 0);
	auto mask_size = cv::Size(mask.cols, mask.rows);

	if (oc_set.find(OC_lane) != oc_set.end())
	{
		LINE_VEC& line_vec = lines_map[OC_lane];
		cv::Mat t = mask.clone();
		if (mask_type != 0)
		{
			cv::inRange(mask, 2, 3, t);
		}
		CenterLine cl;
		cl.getCenterLine(t, line_vec, 1);
		//recoverImageSize(t);	
		recoverContourSize(mask_size, line_vec);
	}
	if (mask_type == 0)
	{
		return;
	}
	for (auto itr = oc_set.begin(); itr != oc_set.end(); itr++)
	{
		if (*itr == OC_lane)
		{
			continue;
		}
		cv::Mat img_binary;
		cv::compare(mask, *itr, img_binary, cv::CMP_EQ);
		LINE_VEC line_vec;
		cv::findContours(img_binary, line_vec, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		if (line_vec.size() == 0)
		{
			continue;
		}
 		recoverContourSize(mask_size, line_vec);
		lines_map.insert(make_pair(*itr, line_vec));
	}

}

void Calib::perspectiveMappingPoints(vector<vector<cv::Point>>& line_pts,  const cv::Mat& warpmat_ipm2src)
{
	vector<cv::Point2f> contoursf;
	vector<cv::Point2f> t;
	for_each(line_pts.begin(), line_pts.end(), [&](auto& cont) {
		contoursf.resize(cont.size());
		transform(cont.begin(), cont.end(), contoursf.begin(),[](const auto& p)->cv::Point2f{
			cv::Point2f fp = p;
			return fp;
		});
		cv::perspectiveTransform(contoursf, t, warpmat_ipm2src);

		cont.clear();
		cont.push_back(cv::Point(t.front()));
		for (int i = 1; i < t.size(); i++)
		{
			cv::Point p = cv::Point(t[i]);
			if (p == cont.back())
			{
				continue;
			}
			cont.push_back(p);
		}
		
	});

}


void Calib::inversePerspectiveMapping(const cv::Mat& src, const cv::Mat& warpmat_src2ipm, cv::Mat& dest)
{
	if (!src.data)
	{
		return;
	}
	float roi_height = 30000;
	float roi_width = 3750;

	//逆透视变换的宽度
	float ipm_width = 800;
	//	float ipm_height = 450;
	float N = 5;
	//保证逆透视变换的宽度大概为5个车头宽
	float scale = (ipm_width / N) / roi_width;
	float ipm_height = roi_height * scale;

	dest = cv::Mat::zeros(ipm_height, ipm_width, src.type());
	cv::warpPerspective(src, dest, warpmat_src2ipm, dest.size());

}

bool Calib::buildKDTree(const map<int, Point_VEC>& ij_linevec, map<int, cv::Mat>& src_map)
{
	bool empty_tree = false;

	auto itr_type = ij_linevec.begin();
	for (; itr_type != ij_linevec.end(); itr_type++)
	{
		const auto& type = itr_type->first;
		const auto& ij_vec = itr_type->second;

		empty_tree |= ij_vec.size() == 0;
		if (empty_tree)
		{
			continue;
		}
		cv::Mat&  source = src_map[type];
		source = cv::Mat(ij_vec).reshape(1);
		source.convertTo(source, CV_32F);

		global_kdtree_map[type].build(source, cv::flann::KDTreeIndexParams(1), cvflann::FLANN_DIST_EUCLIDEAN);
	}

	return !empty_tree;
}

void Calib::releaseKDTree()
{
	auto itr_type = global_kdtree_map.begin();
	for (; itr_type != global_kdtree_map.end(); itr_type++)
	{
		auto& tree = itr_type->second;
		tree.release();
	}
	global_kdtree_map.clear();
}

bool Calib::isValid(const vector<double>&  cam, const vector<double>&  base, const vector<double>& diff)
{
	for (int i = 0; i < 3; i++)
	{
		if (abs(cam[i] - base[i]) > diff[i])
		{
			return false;
		}
	}

	return true;
}

bool isNoChange(const map<int, Point3f_VEC>& xyz_linevec, const vector<double>& v1, const vector<double>& v2)
{
	if (v1.size() != v2.size())
	{
		return true;
	}

	bool no_change = true;
	for (int i = 0; i < v1.size(); i++)
	{
		no_change &= (v1[i] == v2[i]);
	}
	if (no_change)
	{
		return true;
	}

	double diff = 0;
	int diff_sz = 0;
	auto itr_type = xyz_linevec.begin();
	for (; itr_type != xyz_linevec.end(); itr_type++)
	{
		const auto& type = itr_type->first;
		const auto& xyz_vec = itr_type->second;
		vector<cv::Point> lidar_2d_pts;

		auto itr_3d = xyz_vec.begin();
		int i = 0;
		
		for (; itr_3d != xyz_vec.end(); itr_3d++, i++)
		{
			double d3[3] = { itr_3d->x, itr_3d->y, itr_3d->z };
			double d1[2] = {0,0};
			if (!OptimizeCeres::convertPoint3dTo2d(v1, d3, d1))
			{
				continue;
			}
			cv::Point dd1(d1[0], d1[1]);

			double d2[2] = { 0,0 };
			if (!OptimizeCeres::convertPoint3dTo2d(v2, d3, d2))
			{
				continue;
			}
			cv::Point dd2(d2[0], d2[1]);

			diff += cv::norm(dd1 - dd2);
			diff_sz++;
		}
	}
	if (diff_sz == 0)
	{
		return false;
	}
	diff /= diff_sz;
	return diff <2;
}

float isNoChangeSimilarity(const map<int, Point3f_VEC>& xyz_linevec,
	const cv::Mat& blur_mat,
	const vector<double>& v1, const vector<double>& v2)
{
	if (v1.size() != v2.size())
	{
		return true;
	}

	bool no_change = true;
	for (int i = 0; i < v1.size(); i++)
	{
		no_change &= (v1[i] == v2[i]);
	}
	if (no_change)
	{
		return true;
	}

	int count_non_zero1 = 0;
	int count_non_zero2 = 0;
	auto itr_type = xyz_linevec.begin();
	for (; itr_type != xyz_linevec.end(); itr_type++)
	{
		const auto& type = itr_type->first;
		const auto& xyz_vec = itr_type->second;
	
		auto itr_3d = xyz_vec.begin();
		int i = 0;

		for (; itr_3d != xyz_vec.end(); itr_3d++, i++)
		{
			double d3[3] = { itr_3d->x, itr_3d->y, itr_3d->z };
			double d1[2] = { 0,0 };
			if (!OptimizeCeres::convertPoint3dTo2d(v1, d3, d1, false))
			{
				continue;
			}
			cv::Point dd1(d1[0], d1[1]);

			double d2[2] = { 0,0 };
			if (!OptimizeCeres::convertPoint3dTo2d(v2, d3, d2, false))
			{
				continue;
			}
			cv::Point dd2(d2[0], d2[1]);

			if (dd1.x < 0 || dd1.y < 0 ||
				dd2.x < 0 || dd2.y < 0)
			{
				continue;
			}
			uchar grey1 = blur_mat.at<uchar>(dd1.y, dd1.x);
			uchar grey2 = blur_mat.at<uchar>(dd2.y, dd2.x);

			if (grey1 > 0)
			{
				count_non_zero1++;
			}
			 if (grey2 > 0)
			 {
				 count_non_zero2++;
			 }
		}
	}

	if (count_non_zero2 > count_non_zero1)
	{
		return true;
	}
}


void convLines2Points(const map<int, LINE_VEC>& lines_map, 
	map<int, Point_VEC>& ij_vec_map)
{
	for_each(lines_map.begin(), lines_map.end(), [&](const auto& _pair) {
		const auto& line_vec = _pair.second;
		auto& ij_vec = ij_vec_map[_pair.first];
		for_each(line_vec.begin(), line_vec.end(), [&](const auto& line) {
			copy(line.begin(), line.end(), back_inserter(ij_vec));
			});
		});
}


double Calib::iterateClosestPoint2d3d(const map<int, Point3f_VEC>& xyz_vec,
	const map<int, LINE_VEC>& lines_map,
	const map<int, LINE_VEC>& camera_lines_map, 
	vector<double>& camPose, const vector<double>&  diff_threshold)
{
	//vector<double> diff = { 1, 1, 2, 0.1, 0.1, 0.1 };
	vector<double> diff = diff_threshold;
	vector<double> init_campose = camPose;

	map<int, Point_VEC>ij_vec_map;
	convLines2Points(lines_map, ij_vec_map);

	map<int, cv::Mat> src_map;
	if (!buildKDTree(ij_vec_map, src_map))
	{
		return 1000;
	}

	vector<CalibSpace::Point3d2d> p3d2ds;
	//迭代次数
	int total_num = 12;
	int cycle = 4;
	int itr_num = 0;
	double resdual = -1;
	bool no_change = false;

	while (itr_num < total_num)
	{
		p3d2ds.clear();
		double D = calcD(itr_num, cycle);
		//double D = 50;
		int tm = itr_num / cycle;
		findCorrespondPoints(camPose, xyz_vec, ij_vec_map, p3d2ds, D, tm);
		if (p3d2ds.size() < 4)
		{
			break;
		}

		bool  valid = false;
		if (CalibSpace::camera_type == CAMERA_MSS_WIDE)
		{
			vector<double> pnp_camPose = camPose;
			vector<int> inliers;
			if (optimizePnP(p3d2ds, pnp_camPose, inliers, true) &&
				isValid(pnp_camPose, init_campose, diff/*_threshold*/))
			{
				camPose.swap(pnp_camPose);
				valid = true;

#if 1
				no_change = isNoChange(xyz_vec, pnp_camPose, camPose);
				//no_change = isNoChangeSimilarity(xyz_vec, blur_mat,pnp_camPose, camPose);
				if (no_change)
				{
					break;
				}
#endif
			}
		}
		///////////////////////
		if (!valid)
		{
			vector<double> ceres_camPose = camPose;
			OptimizeCeres::optimize(p3d2ds, ceres_camPose);
			if (isValid(ceres_camPose, init_campose, diff/*_threshold*/))
			{
				camPose.swap(ceres_camPose);
				valid = true;
#if 1
				no_change = isNoChange(xyz_vec, ceres_camPose, camPose);
				//no_change = isNoChangeSimilarity(xyz_vec, blur_mat, ceres_camPose, camPose);
				if (no_change)
				{
					break;
				}
#endif
			}
		}

		itr_num++;
		
	}
	releaseKDTree();

	LOG(INFO) << ("iterate Xs,Ys,Zs,omega,pho,kappa = ");
	for (int i = 0; i < 6; i++)
	{
		LOG(INFO) << ("%.3f",camPose[i]);
	}

	return resdual;
}

bool Calib::findCorrespondPoints(const vector<double>& camRot, const map<int, Point3f_VEC>& xyz_linevec,
	const map<int, Point_VEC>& ij_linevec,
	vector<CalibSpace::Point3d2d>& p3d2ds, 
	float D,
	int tm)
{
	vector<vector<cv::Point>> match_line_vec;
	vector<vector<cv::Point>> reverse_match_line_vec;

	auto itr_type = xyz_linevec.begin();
	for (; itr_type != xyz_linevec.end(); itr_type++)
	{
		const auto& type = itr_type->first;
		//if (H_ASSGIN &&tm == 3 && (type < -1 || type > 1))
		//{
		//	continue;
		//}
		const auto& xyz_vec = itr_type->second;
		auto find_ij = ij_linevec.find(type);
		if (find_ij == ij_linevec.end())
		{
			continue;
		}
		const auto& ij_vec = find_ij->second;
		auto find_tree = global_kdtree_map.find(type);
		if (find_tree == global_kdtree_map.end())
		{
			continue;
		}
		auto& tree = find_tree->second;

		vector<cv::Point> lidar_2d_pts;
		vector<int>  lidar_2d_indices;
		auto itr_3d = xyz_vec.begin();
		int i = 0;
		for (; itr_3d != xyz_vec.end(); itr_3d++, i++)
		{
			double d3[3] = { itr_3d->x, itr_3d->y, itr_3d->z };
			double d2[2];
			if (!OptimizeCeres::convertPoint3dTo2d(camRot, d3, d2, true))
			{
				continue;
			}

			cv::Point dd2(d2[0], d2[1]);

			lidar_2d_pts.push_back(dd2);
			lidar_2d_indices.push_back(i);

			int idx = -1;

			//	double DD = calcDD(D, dd2);
			double DD = D;
			//LOG(INFO) << D << "," << DD;

			double nearest_dist = 1000;
			bool search_result = searchNeareatPoint(ij_vec, tree, dd2, idx, nearest_dist, DD);
			
			if (!search_result)
			{
				continue;
			}

			cv::Point search_pt = ij_vec[idx];
			cv::Point delta = search_pt - dd2;
			if (	abs(delta.x * 1.0 / delta.y) < 2)
			{
				continue;
			}
			
			double tmp[2] = { dd2.x, dd2.y };
			OptimizeCeres::IPM2Image(tmp);
			dd2 = cv::Point2d(tmp[0], tmp[1]);

			tmp[0] = search_pt.x; tmp[1]= search_pt.y;
			OptimizeCeres::IPM2Image(tmp);
			search_pt = cv::Point2d(tmp[0], tmp[1]);


			vector<cv::Point> line(2);
			line[0] = dd2;
			line[1] = search_pt;
			match_line_vec.push_back(line);

			CalibSpace::Point3d2d p3d2d;
			p3d2d.p3d = *itr_3d;
			p3d2d.p2d = search_pt;

			p3d2ds.push_back(p3d2d);
		}

#if 1
		//建树
		if (lidar_2d_pts.size() > 0)
		{
			cv::Mat  mat_lidar_2d = cv::Mat(lidar_2d_pts).reshape(1);
			mat_lidar_2d.convertTo(mat_lidar_2d, CV_32F);
			cv::flann::Index tree = cv::flann::Index(mat_lidar_2d, cv::flann::KDTreeIndexParams(1));
			auto itr_2d = ij_vec.begin();
			i = 0;
			for (; itr_2d != ij_vec.end(); itr_2d++, i++)
			{
				cv::Point d2 = *itr_2d;
				int idx = -1;
				//	double DD = calcDD(D, dd2);
				double DD = D;

				double nearest_dist = 1000;
				if (!searchNeareatPoint(lidar_2d_pts, tree, d2, idx, nearest_dist, DD))
				{
					continue;
				}

				cv::Point search_pt = lidar_2d_pts[idx];

				cv::Point delta = search_pt - d2;
				if (abs(delta.x * 1.0 / delta.y) < 2)
				{
					continue;
				}

				double tmp[2] = { d2.x, d2.y };
				OptimizeCeres::IPM2Image(tmp);
				d2 = cv::Point2d(tmp[0], tmp[1]);

				tmp[0] = search_pt.x; tmp[1] = search_pt.y;
				OptimizeCeres::IPM2Image(tmp);
				search_pt = cv::Point2d(tmp[0], tmp[1]);

				vector<cv::Point> line(2);
				line[0] = d2;
				line[1] = search_pt;
				reverse_match_line_vec.push_back(line);

				CalibSpace::Point3d2d p3d2d;
				p3d2d.p3d = xyz_vec[lidar_2d_indices[idx]];
				p3d2d.p2d = d2;

				p3d2ds.push_back(p3d2d);
			}
			tree.release();
		}

#endif
	}


	//第一次匹配的时候保存一下
	if (0)
	{
		cv::Mat match_mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC3, cv::Scalar(0, 0, 0));
	//	cv::Mat match_mat(1280, 800, CV_8UC3, cv::Scalar(0, 0, 0));
		if (m_camera_img.rows > 0)
		{
			match_mat = m_camera_img.clone();
		}
//		cv::cvtColor(match_mat, match_mat, cv::COLOR_GRAY2BGR);

		cv::polylines(match_mat, reverse_match_line_vec, false, cv::Scalar(255, 0, 0), 2);
		cv::polylines(match_mat, match_line_vec, false, cv::Scalar(0, 0, 255), 2);

		//cv::polylines(match_mat, m_lidar_2d_contours, false, cv::Scalar(255, 0, 0), 2);

		string temp_path = "kdtree/";
		cv::imwrite(temp_path + to_string(int(D)) + ".png", match_mat);
	}

	return p3d2ds.size() > 0;
}

bool Calib::optimizePnP(const vector<CalibSpace::Point3d2d>& ref_vec, vector<double>& camPose, vector<int>& inliers, bool ransac)
{
	CCalib mc;

	cv::Mat rvec;
	cv::Mat tvec;
	if (!mc.solvePnP(ref_vec, CalibSpace::intrisicMat, CalibSpace::distCoeffs, rvec, tvec, inliers, ransac))
	{
		return false;
	}

	mc.rt2camPose(rvec, tvec, camPose);
	return true;
}



void changeDistanceTransform(cv::Mat mat, int h, cv::Mat& blur_mat)
{
#if 1
	int h0 = CalibSpace::image_rect.height / 2.0;
	
	
	cv::Rect rect;
	cv::Mat tmp;
	
	int y = h0;
	
	rect = cv::Rect(0, y, CalibSpace::image_rect.width, h);

	mat(rect).copyTo(tmp);
	auto kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(10, 10));
	cv::dilate(tmp, tmp, kernel);
	tmp.copyTo(mat(rect));

	y = y + h;
	rect = cv::Rect(0, y, CalibSpace::image_rect.width, h0 - h);
	mat(rect).copyTo(tmp);
	kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21));
	cv::dilate(tmp, tmp, kernel);
	tmp.copyTo(mat(rect));


	//y = h + h * 2.0 / 3.0;
	//rect = cv::Rect(0, y, CalibSpace::IMG_WIDTH, int(h / 3.0));
	//mat(rect).copyTo(tmp);
	//kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
	//cv::dilate(tmp, tmp, kernel);
	//tmp.copyTo(mat(rect));

	//cv::imwrite("0_mat.jpg", mat);
#else
	//kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(21, 21));
	//cv::dilate(mat, mat, kernel);
#endif
	//cv::distanceTransform(mat, blur_mat, DIST_L1, 3);
	cv::distanceTransform(mat, blur_mat, DIST_L2, 5);
}

double Calib::iterateCrossEntropy(const map<int, Point3f_VEC>& xyz_vec_map,
	const map<int, LINE_VEC>& lines_map, 
	const float& pitch,
	vector<double>& camPose)
{
	cv::Mat mat = cv::Mat(CalibSpace::image_rect.height, CalibSpace::image_rect.width, CV_8UC1, cv::Scalar(0));

	auto itr = lines_map.begin();
	for (; itr != lines_map.end(); itr++)
	{
		const auto& type = itr->first;
		const auto& line_vec = itr->second;
		
		//cv::polylines(mat, line_vec, false, cv::Scalar(255), 50);
		
		cv::polylines(mat, line_vec, false, cv::Scalar(255), 10);
	}
	cv::Mat blur_mat;
	//mat.convertTo(blur_mat, CV_32F, 1.0/255);
	
	//cv::distanceTransform(mat, blur_mat, DIST_L1, 3);
	//cv::distanceTransform(mat, blur_mat, DIST_L2, 5);
	int h = 150;
	if (abs(pitch) >= 1.0 / 180 * M_PI)
	{
		h = 50;
	}
	changeDistanceTransform(mat, h, blur_mat);

	//cv::normalize(blur_mat, blur_mat, 0, 255, cv::NORM_MINMAX);

	cv::normalize(blur_mat, blur_mat, 0, 1, cv::NORM_MINMAX);

	//cv::imwrite("0_dt.jpg", blur_mat);

	//cout << blur_mat;
	OptimizeCeres::optimizeCrossEntropy(xyz_vec_map, blur_mat, camPose);
	return 0;
}

void Calib::setCameraImage(const cv::Mat& img)
{
	m_camera_img = img;
}

