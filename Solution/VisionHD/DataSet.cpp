#include "DataSet.h"
#include <iostream>
#include <opencv2/imgproc.hpp>

namespace CalibSpace
{
	//kbd cs
	int band = 0;
	double CX = 0;
	double CY = 0;
	double FX = 0;
	double FY = 0;
	int IMG_WIDTH = 0;
	int IMG_HEIGHT = 0;
	cv::Mat warpmat_src2ipm;
	cv::Mat warpmat_ipm2src;
	CameraType camera_type = CAMERA_MSS_PANO;
	cv::Rect image_rect;

	cv::Mat intrisicMat;
	cv::Mat_<double> distCoeffs;
	
	vector<double> extrinsic_para(6);
	float ego_height = 0;

	bool activate_flg = true;
	void EigenTranslateAndRot(const Eigen::Vector3d& pt, Eigen::Vector3f& lpt, const  Eigen::Vector3d& t, const  Eigen::Matrix3f& r)
	{
		for (int i =0; i < 3; i++)
		{
			lpt[i] = float(pt[i] - t[i]);
		}
	//	cout << lpt << endl;
		lpt = r * lpt;
	//	cout << lpt << endl;

	}

	void TranslateAndRot(const cv::Point3d& pt, cv::Point3d& lpt, const cv::Point3d& t, const cv::Mat& r)
	{
		lpt = pt - t;
		cv::Mat mat(lpt);
//		std::cout << mat << endl;
		mat = r * mat;
//		std::cout << mat << endl;
		lpt.x = mat.at<double>(0);
		lpt.y = mat.at<double>(1);
		lpt.z = mat.at<double>(2);
	}

	void RotAndTranslate(const cv::Point3d& pt, cv::Point3d& pca, const cv::Point3d& t, const cv::Mat& r)
	{
		cv::Point3d temp = pt ;
		cv::Mat mat(temp);
		mat = r * mat;
		temp.x = mat.at<double>(0);
		temp.y = mat.at<double>(1);
		temp.z = mat.at<double>(2);

		temp = temp + t;
		pca = temp;
	}

	void Camera2Image(const cv::Point3d& pca, cv::Point& pp)
	{
		pp.x = CalibSpace::FX * pca.x / pca.z + CalibSpace::CX;
		pp.y = CalibSpace::FY * pca.y / pca.z + CalibSpace::CY;
	}

	void initInversePerspectiveMappingMat(const vector<cv::Point2f>& corners,
		cv::Mat& warpmat_src2ipm,
		cv::Mat& warpmat_ipm2src)
	{
		//float roi_height = 30000;
		float roi_height = 60000;
		float roi_width = 3750;

		//逆透视变换的宽度
		float ipm_width = 800;
		//	float ipm_height = 450;
		float N = 5;
		//保证逆透视变换的宽度大概为5个车头宽
		float scale = (ipm_width / N) / roi_width;
		float ipm_height = roi_height * scale;

		vector<cv::Point2f> corners_trans(4);
		corners_trans[0] = cv::Point2f(ipm_width / 2 - ipm_width / (2 * N), ipm_height * 0.1);
		corners_trans[1] = cv::Point2f(ipm_width / 2 + ipm_width / (2 * N), ipm_height * 0.1);
		corners_trans[2] = cv::Point2f(ipm_width / 2 - ipm_width / (2 * N), ipm_height * 0.8);
		corners_trans[3] = cv::Point2f(ipm_width / 2 + ipm_width / (2 * N), ipm_height * 0.8);
		//计算原图到逆透视图和逆透视图到原图的变换矩阵
		warpmat_src2ipm = cv::getPerspectiveTransform(corners, corners_trans);
		warpmat_ipm2src = cv::getPerspectiveTransform(corners_trans, corners);

	}
}
