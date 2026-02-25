// ManualCalib.cpp : 定义控制台应用程序的入口点。
// for std
#include <iostream>
// for opencv
#include "CCalib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include<fstream>
#include <glog/logging.h>


#include "Optimize.h"
#include "HNMath/TransRotation.h"
#include "DataIO.h"
#include "TestCalib.h"

CCalib::CCalib()
{
}

CCalib::~CCalib()
{
}

void CCalib::initCamera(cv::Mat& intrisicMat, cv::Mat_<double>& distCoeffs)
{
	intrisicMat = cv::Mat(3, 3, cv::DataType<double>::type); // Intrisic matrix
	distCoeffs = cv::Mat_<double>(1, 5, cv::DataType<double>::type);   // Distortion vector
	intrisicMat.at<double>(0, 0) = CalibSpace::FX;
	intrisicMat.at<double>(1, 0) = 0;
	intrisicMat.at<double>(2, 0) = 0;

	intrisicMat.at<double>(0, 1) = 0;
	intrisicMat.at<double>(1, 1) = CalibSpace::FY;
	intrisicMat.at<double>(2, 1) = 0;

	intrisicMat.at<double>(0, 2) = CalibSpace::CX;
	intrisicMat.at<double>(1, 2) = CalibSpace::CY;
	intrisicMat.at<double>(2, 2) = 1;

	distCoeffs << 0.0, 0.0, 0.0, 0.0, 0.0;
}

bool CCalib::solvePnP(const vector<CalibSpace::Point3d2d>& ref_pt_vec,
	const cv::Mat& intrisicMat, const cv::Mat_<double> & distCoeffs, cv::Mat& rVec, cv::Mat& tVec, vector<int>& inliers, bool ransac)
{
	if (ref_pt_vec.size() < 3)
	{
		return false;
	}
	vector<cv::Point3d> d3_pts(ref_pt_vec.size());
	vector<cv::Point2d> d2_pts(ref_pt_vec.size());

	transform(ref_pt_vec.begin(), ref_pt_vec.end(), d3_pts.begin(), [](const auto& p3d2d)->cv::Point3d {
		return p3d2d.p3d; });

	transform(ref_pt_vec.begin(), ref_pt_vec.end(), d2_pts.begin(), [](const auto& p3d2d)->cv::Point2d{

		return p3d2d.p2d; 	
		});
	if (ransac)
	{
		return cv::solvePnPRansac(d3_pts, d2_pts, intrisicMat, distCoeffs, rVec, tVec, false, 100, 8.0, 0.99, inliers);
	}
	else
	{
		return cv::solvePnP(d3_pts, d2_pts, intrisicMat, distCoeffs, rVec, tVec);
	}
}

Eigen::Vector3f getEuler(const Eigen::Matrix3f& rr)
{
	Eigen::Vector3f euler = rr.eulerAngles(0, 1, 2);
	Eigen::Vector3f eulerAngle_mine;
	Eigen::Matrix3f rot = rr;
	eulerAngle_mine(2) = std::atan2(rot(2, 1), rot(2, 2));
	eulerAngle_mine(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
	eulerAngle_mine(0) = std::atan2(rot(1, 0), rot(0, 0));

	euler = eulerAngle_mine;
	return euler;
}


void CCalib::rt2camPose(const cv::Mat& rVec, const cv::Mat& tVec, vector<double>& camPose)
{
	camPose[0] = tVec.at<double>(0);
	camPose[1] = tVec.at<double>(1);
	camPose[2] = tVec.at<double>(2);

	cv::Mat R;
	cv::Rodrigues(rVec, R);


	Eigen::Matrix3f ER;
	cv::cv2eigen(R, ER);
//	cout << ER << endl;
#if 0
	Eigen::Vector3f euler = ER.eulerAngles(0, 1, 2);
	camPose[3] = euler[2];
	camPose[4] = euler[1];
	camPose[5] = euler[0];
#else
	Eigen::Vector3f euler = getEuler(ER);
	camPose[3] = euler[0];
	camPose[4] = euler[1];
	camPose[5] = euler[2];
#endif


}

///////////////////////////////////////////ransac///////////////////////////////////////////////////////
