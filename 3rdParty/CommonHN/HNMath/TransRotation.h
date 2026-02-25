#pragma once
#ifndef TransRotation_H
#define TransRotation_H

#include "CommonHN.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/core.hpp>

class COMMONHN_API TransRotation 
{
public:
	//四元数转欧拉角
	static void eigenQuaternion2EulerAngle(const Eigen::Quaternionf& q, float& yaw, float& pitch, float& roll);
	static void eigenEulerAngle2Quaternion(const double& yaw, const double& pitch, const double& roll, Eigen::Quaterniond& q);
	//旋转矩阵到欧拉角
	static void eigenRotationMatrix2EulerAngles(const Eigen::Matrix3d& R, Eigen::Vector3d& euler);

	//欧拉角到旋转矩阵
	//static void eulerAngles2RotationMatrix(const cv::Vec3d &theta, cv::Mat& R);
	static void eigenEuler2RotationMatrix(const Eigen::Vector3f& euler, Eigen::Matrix3f& rotation_matrix);
	static void eigenEuler2RotationMatrixd(const Eigen::Vector3d& euler, Eigen::Matrix3d& rotation_matrix);

	//计算欧拉角差值
	static void eigenEulerDiff(const Eigen::Vector3d& euler1, const Eigen::Vector3d& euler2, Eigen::Vector3d& euler_dif);
	////旋转矩阵到欧拉角
	//static void rotationMatrix2EulerAngles(const cv::Mat &R, cv::Vec3f&euler);

	////rodrigues到旋转矩阵
	static void rodrigues2RotationMatrix(const cv::Mat &rVec, cv::Mat &R);
	//static void rotationMatrix2Rodrigues(const cv::Mat &R, cv::Mat &rVec);
};

#endif