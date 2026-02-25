#pragma once
#ifndef DATASET_H
#define DATASET_H
//#include "mrpt/utils/types_math.h"
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <vector>
#include <string>
#include <map>

using namespace std;

typedef vector<cv::Point3f> Point3f_VEC;
typedef vector<cv::Point> Point_VEC;
typedef vector<vector<cv::Point>> LINE_VEC;


enum CameraType {
	CAMERA_MSS_WIDE = 0,
	CAMERA_MSS_PANO,
};

namespace CalibSpace{
	extern int band;
	extern double CX;
	extern double CY;
	extern double FX;
	extern double FY;
	extern int IMG_WIDTH;
	extern int IMG_HEIGHT;
	extern 	cv::Mat intrisicMat;
	extern cv::Mat_<double> distCoeffs;
	extern CameraType camera_type;
	extern cv::Rect image_rect;
	extern vector<double> extrinsic_para;
	extern float ego_height;
	extern cv::Mat warpmat_src2ipm;
	extern cv::Mat warpmat_ipm2src;
	extern bool activate_flg;
	class Point3d2d
	{
	public:
		Point3d2d() {};
		Point3d2d(const cv::Point3d& _p3d, const cv::Point2d& _p2d) { p3d = _p3d; p2d = _p2d; type = 0; res = -1; }
		~Point3d2d() {};

		cv::Point3d p3d;
		cv::Point2d p2d;
		int type;//¿‡–Õ
		double res;
	};

	class PointXYZI
	{
	public:
		PointXYZI() {};
		PointXYZI(const cv::Point3d& pt, const double& _intensity) {
			x = pt.x;
			y = pt.y;
			z = pt.z;
			intensity = _intensity;
		};
		~PointXYZI() {};

		double x;
		double y;
		double z;
		double intensity;
	};	
	void TranslateAndRot(const cv::Point3d& pt, cv::Point3d& lpt, const cv::Point3d& t, const cv::Mat& r);
	void RotAndTranslate(const cv::Point3d& pt, cv::Point3d& pca, const cv::Point3d& t, const cv::Mat& r);
	void Camera2Image(const cv::Point3d& pca, cv::Point& pp);

	void EigenTranslateAndRot(const Eigen::Vector3d& pt, Eigen::Vector3f& lpt, const  Eigen::Vector3d& t, const  Eigen::Matrix3f& r);

	void initInversePerspectiveMappingMat(const vector<cv::Point2f>& corners,
		cv::Mat& warpmat_src2ipm,
		cv::Mat& warpmat_ipm2src);
}


class TrajectoryData
{
public:
	double time;
	string name;
	cv::Point3d point;
	cv::Point3d lonlat;
	double roll;
	double pitch;
	double heading;
};

class HDMapPointID
{
public:
	string obj_id;
	int v_id;

	bool operator<(const HDMapPointID& a) const
	{
		if (obj_id != a.obj_id)
		{
			return obj_id < a.obj_id;
		}
		else
		{
			return v_id < a.v_id;
		}
	}

};

class HDObject
{
public:
	HDObject() {};
	~HDObject() {};

	string obj_id;
	string prop_id;
	int type;
	int horizonal_idx;
	vector<cv::Point3d> shape;
	vector<cv::Point3d> shape_org;
	cv::Rect2d rect_3d;
	//---add 2d
	vector<cv::Point> ij_shape;
	cv::Rect rect;
};
typedef vector<HDObject> HDObject_VEC;

inline string getFileName(const string& path)
{
	auto spos = path.rfind('\\');
	if (spos == string::npos)
	{
		spos = path.rfind('/');
	}
	if (spos == string::npos)
	{
		return "";
	}
	auto epos = path.rfind('.');
	string file_na = path.substr(spos + 1, epos - spos - 1);
	return file_na;
}


inline void getImageName(string& sta_name, string& name)
{
	auto rmv_end = name.find(".jpg");
	if (rmv_end != string::npos)
	{
		name = name.substr(0, rmv_end);
	}
	auto spos = name.rfind('@');
	if (spos == string::npos)
	{
		return;
	}
	sta_name = name.substr(0, spos);
	string img_na = name.substr(spos + 1, name.size() - spos - 1);
	name =  img_na;
}

enum ObjectClassification
{
	OC_road = 1,
	OC_lane = 13,
	OC_car,
};

struct CamPose
{
	string img_na;
	int idx;
	TrajectoryData ins;
	vector<double> camPose;
	float regist_probability;
	double res;
	bool regist_flg;

	Eigen::Vector4d ins_q;
	Eigen::Vector4d q;

};

typedef HDObject Object3D;
typedef vector<Object3D> Object3D_VEC;
#endif