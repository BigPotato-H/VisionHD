// Calib.cpp : 定义控制台应用程序的入口点。

// for std
// for opencv
#pragma once
#ifndef Calib_H
#define Calib_H
//#include "RansacSolver.h"

#include "DataSet.h"
//#include <Eigen\src\Geometry\AngleAxis.h>
#include <set>
#include <opencv2/flann/miniflann.hpp>
#include <Eigen/Core>

//#include "Kalman.h"

enum SolveMethod {
	NONE = -1,
	ICP = 0,
	JC = 1
};

class Calib
{
public:
	Calib();
	~Calib();

	void setCameraImage(const cv::Mat& img);

	void calcBlurImage(const vector<vector<cv::Point>>& line_vec, cv::Mat& blur_mat);
	void calcBlurImage(const map<int, LINE_VEC>& line_vec_map, cv::Mat& blur_mat);

	void calcDistanceTransformImage(const map<int, LINE_VEC>& line_vec_map,
		cv::Mat& blur_mat);
	
	void extractImageDeepLearningMultiLines(const string& folder_path,
		const string& img_na,
		const set<ObjectClassification>& oc_set, map<int, LINE_VEC>& lines_map, int mask_type = 0);

	void inversePerspectiveMapping(const cv::Mat& src, const cv::Mat& warpmat_src2ipm, cv::Mat& dest);
	bool buildKDTree(const map<int, Point_VEC>& ij_linevec, map<int, cv::Mat>& src_map);
	void releaseKDTree();
	bool isValid(const vector<double>&  cam, const vector<double>&  base, const vector<double>&  diff);
	double iterateClosestPoint2d3d(const map<int, Point3f_VEC>& xyz_vec,
		const map<int, LINE_VEC>& lines_map,
		const map<int, LINE_VEC>& camera_lines_map,
		vector<double>& camPose, const vector<double>& diff_threshold);

	double iterateCrossEntropy(const map<int, Point3f_VEC >& xyz_vec_map,
		const map<int, LINE_VEC>& lines_map, 
		const float& pitch,
		vector<double>& camPose);
	bool findCorrespondPoints(const vector<double>& camRot, 
		const map<int, Point3f_VEC>& xyz_vec, 
		const map<int, Point_VEC>& ij_vec, 
		vector<CalibSpace::Point3d2d>& p3d2ds, 
		float D,
		int tm);
	
	bool optimizePnP(const vector<CalibSpace::Point3d2d>& ref_vec, 
		vector<double>& camPose, 
		vector<int>& inliers, 
		bool ransac = false);

	void perspectiveMappingPoints(vector<vector<cv::Point>>& line_pts, const cv::Mat& warpmat_ipm2src);

private:

	cv::Mat m_camera_img;
	//cv::flann::Index global_kdtree;
	map<int, cv::flann::Index> global_kdtree_map;

};

#endif