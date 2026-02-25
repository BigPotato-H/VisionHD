#pragma once
#ifndef CalibMSS_H
#define CalibMSS_H

#include "DataSet.h"
#include "TestCalib.h" 
#include "DataIO.h"
#include "KF/FusionEKF.h"


#ifdef IMAGEMATCH_EXPORTS
#define IMAGEMATCH_API   __declspec(dllexport)
#else
#define IMAGEMATCH_API __declspec(dllimport)
#endif
#include <string>

//#define DS_LOG //delicate system car online
#define  READ_MASK 1  //read mask or xml
#define CONFIG_PARA_TBL "config_para_tbl"
#define CONFIG_PARA_RECON_TBL "config_para_recon_tbl"



enum RegMethod
{
	//0 none
	// 10 bmm
	//100 dst
	//1000ekf
	//RegMethod_NONE = 0,
	//RegMethod_BMM = 2,  //bmm
	//RegMethod_BMM_EKF=10,  //bmm
	//RegMethod_DST=4, //距离变换
	//RegMethod_DST_EKF=12 //距离变换s

	RegMethod_NONE = 0,
	RegMethod_BMM = 1,  //bmm
	RegMethod_BMM_EKF = 2,  //bmm

	RegMethod_NONE_GT = 5,
	RegMethod_CE_EKF = 6 //cross entropy
};

class HDMapData
{
public:
	HDMapData() {};
	~HDMapData() {};

	HDObject_VEC ld_vec;
	HDObject_VEC hd_obj_vec;
	HDObject_VEC junction_vec;
};

class IMAGEMATCH_API AlignImageHD
{
public:
	AlignImageHD();
	~AlignImageHD();

	void process(int step = 0);

	bool preprocess(vector<TrajectoryData>& ins_vec,
		HDMapData& map_data,
		const std::string & _dataPath,
		const std::string & _midPath,
		const int& band, const string& para_tbl_na = CONFIG_PARA_TBL,
		string car_id = "");

	void preprocess(int step, int method);
	void jitterCamPose(map<string, CamPose>& pose_map);

	void groudTruthCamPose(map<string, CamPose>& pose_map);
	void processRegistHDAndMSSImages();

	bool isInIntersection(CamPose& cp);
	void kalmanFilterInitialize(const vector<double>& pose);
	bool kalmanFilterEstimate(CamPose& cp);
	void inversePerspectiveAdjustImageLines(const string& img_na, map<int, LINE_VEC>& lines_map);

protected:

	bool initConfig(const string& para_tbl_na,const string& car_id);
	bool initCalibSpace();

	void updateCamPose(vector<double>& campos);
	int getImageIndex(const string& img_na);
	void getImageCenter(cv::Point2f& c);
	void splitLaneContours(LINE_VEC& contoursf);
	void buildTimeIndex(cv::Mat& source, const vector<TrajectoryData>& ins_vec);
	
	bool readTrajecotry(vector<TrajectoryData>& ins_vec);
	void readHDMap(bool keep_inside_intersection = false);
	
	bool getLocalHDMap(const vector<TrajectoryData>& ins_vec, const int& idx, HDObject_VEC& local_local_map_vec);


	bool isValid(const vector<double>&  cam, const vector<double>&  base, const vector<double>&  diff_threshold);

	void undistortImage(const cv::Mat& src_img, cv::Mat& dest_img);
	//for read image semantic by self reading mask
	bool registHDImageLaneDividers(const vector<TrajectoryData>& ins_vec,
		const int& idx,
		CamPose& cp);

	bool optimizePose(map<int, Point3f_VEC>& xyz_vec_map, const map<int, LINE_VEC>& lines_map, 
		const map<int, LINE_VEC>& camera_lines_map,
		const TrajectoryData& ins,
		vector<double>& campos);
	float evaluteSimilarity(const map<int, LINE_VEC>& lines_map_rgb, const map<int, LINE_VEC>& lines_map_hdmap);
	bool matchImageWithLog(const string& img_na, double t0, double t_end, float time_res, int& idx);

	template<typename T> void getEgoMapPoints(const vector<T>& map_vec, const vector<double>& camPose, map<int, Point3f_VEC>& xyz_vec_map);
	template<typename T> void getEgoMapPoints(const vector<T>& map_vec, const vector<double>& camPose, map<int, vector<Point3f_VEC>>& xyz_vec_map);
	//	template<typename T> void getValid3dPoints(const vector<T>& map_vec, const RAW_INS& ins, const double* camPose, vector<cv::Point3f>& xyz_vec, vector<vector<cv::Point>>& contours);
	template<typename T> void getEgoMapContours(const vector<T>& map_vec, const vector<double>& camPose, 
		map<int, LINE_VEC>& contours_map,
		bool ipm = true);
	template<typename T> 
	void transformHDMapFromWorld2Camera(const vector<T>& map_vec, const TrajectoryData& ins, const vector<double>& camPose, 
		map<HDMapPointID, cv::Point3f>& cam_pts_map);
	template<typename T> 
	void transformHDMapFromWorld2Image2(const vector<T>& map_vec, const CamPose& cp, const Eigen::Matrix3f& R, 
		const Eigen::Vector3f& Trans, map<int, LINE_VEC>& contours_map);
	ObjectClassification convHDMapType2ObjectClassification(int type);
template<typename T>
	void transformHDMapFromWorld2Ego(const vector<T>& map_vec, const TrajectoryData& ins, vector<T>& local_local_map_vec);
	void transformInsFromWorld2Ego(const vector<TrajectoryData>& ins_vec,  const int& idx,
		vector<TrajectoryData>& local_ins_vec);
	void convInsFromWorld2Ego(const vector<TrajectoryData>& ins_vec, const TrajectoryData& ins, 
		vector<TrajectoryData>& local_ins_vec);

	template<typename T>
	void transformEgo2Camera(const vector<T>& map_vec, const vector<double>& camPose, map<int, Point3f_VEC>& xyz_vec_map);

	bool beyongImageRange(int type, const Eigen::Vector3f& lp);
	template<typename T> 
	void removeByTraceRange(vector<T>& local_local_map_vec, const vector<TrajectoryData>& ins_vec);
	
	void creatFolder(const string& folder_path);

	void mergeImageLines(vector<vector<cv::Point2f>>& line_vec);
	template<typename T>
	void horizonalAssignHDMap(vector<T>& local_local_map_vec, const vector<TrajectoryData>& local_ins_vec);

	void saveImage(const string& sub_folder,
		const string& img_na, 
		const cv::Mat& img, 
		const cv::Scalar& sca,
		map<int, LINE_VEC>& lines_map);

	template<typename PointT>
	void fitByOpenCV(std::vector<PointT>& img_shp, int n);


	void collectLocalIns(const vector<TrajectoryData>& ins_vec,
		int idx,
		double len,
		vector<TrajectoryData>& local_ins_vec,
		bool add = true,
		bool farward = true);
	void transformCoord(HDObject_VEC& _data);
protected:

	Calib m_calib;
	//cv::Point3f m_tt0;
	//cv::Vec3f m_euler0;
	//vector<double> m_camPose;
	//vector<double> m_diff_threshold;

/* 	vector<double> m_last_camPose;*/
	vector<TrajectoryData> m_ins_vec;
	vector<cv::Point2f> m_trace_box;
	HDMapData m_hd_map;

	set<ObjectClassification> m_reg_oc_set;

	map<string, CamPose> m_pos_map;

	DataIO* m_io;

	FusionEKF* m_kf;

	RegMethod m_reg_method;
	string SUB_FOLDER_PLY;
};

inline void findPeakPoints(const map<double, double>& value_p_map, vector<double>& v_vec, double merge_width)
{
	if (value_p_map.size() == 0)
	{
		return;
	}

	if (value_p_map.size() == 1)
	{
		v_vec.push_back(value_p_map.begin()->first);
	}

	//	double avg_v = 0;
	map<double, double>peak_big_v_map;
	auto itr = value_p_map.begin();
	for (; itr != value_p_map.end(); itr++)
	{
		auto last_itr = itr;
		auto next_itr = itr;
		next_itr++;
		last_itr--;
		//		avg_v += itr->second;

		//首
		if (itr == value_p_map.begin() && itr->second - next_itr->second >= 0)
		{
			peak_big_v_map.insert(*itr);
			continue;
		}
		//中
		if (itr->second - next_itr->second >= 0 &&
			itr->second - last_itr->second >= 0)
		{
			peak_big_v_map.insert(*itr);
		}
		//尾
		if (next_itr == value_p_map.end())
		{
			if (itr->second - last_itr->second >= 0)
			{
				peak_big_v_map.insert(*itr);
			}
			break;
		}
	}
	//	avg_v = avg_v / value_p_map.size();

	itr = peak_big_v_map.begin();
	for (; itr != peak_big_v_map.end();)
	{
		auto last_itr = itr;
		auto next_itr = itr;
		next_itr++;
		last_itr--;
		//if (itr->second < avg_v)
		//{
		//	itr = peak_big_v_map.erase(itr);
		//	continue;
		//}
		//尾
		if (peak_big_v_map.size() == 1)
		{
			break;
		}

		if (next_itr == peak_big_v_map.end())
		{
			if (abs(itr->first - last_itr->first) < merge_width)
			{
				auto v = (itr->first + last_itr->first) / 2;
				auto vp = itr->second;
				peak_big_v_map.erase(itr);
				peak_big_v_map.erase(last_itr);
				peak_big_v_map.insert(make_pair(v, vp));
			}
			break;
		}
		//首
		if (abs(itr->first - next_itr->first) < merge_width)
		{
			auto v = (itr->first + next_itr->first) / 2;
			auto vp = itr->second;
			peak_big_v_map.erase(next_itr);
			itr = peak_big_v_map.erase(itr);
			peak_big_v_map.insert(make_pair(v, vp));
			itr--;
			continue;
		}
		else
		{
			itr++;
		}
	}

	for_each(peak_big_v_map.begin(), peak_big_v_map.end(), [&](const auto& p) {
		v_vec.push_back(p.first);
	});

	return;
}


template<typename PointT>
void AlignImageHD::fitByOpenCV(std::vector<PointT>& img_shp, int n)
{
	//n次多项式
	if (img_shp.size() <= 2)
	{
		return;
	}
	//Number of key points  
	int N = img_shp.size();

	//构造矩阵X  
	cv::Mat X = cv::Mat::zeros(n + 1, n + 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int j = 0; j < n + 1; j++)
		{
			for (int k = 0; k < N; k++)
			{
				X.at<double>(i, j) = X.at<double>(i, j) +
					std::pow(img_shp[k].y, i + j);
			}
		}
	}

	//构造矩阵Y  
	cv::Mat Y = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < N; k++)
		{
			Y.at<double>(i, 0) = Y.at<double>(i, 0) +
				std::pow(img_shp[k].y, i) * img_shp[k].x;
		}
	}

	cv::Mat  A = cv::Mat::zeros(n + 1, 1, CV_64FC1);
	//求解矩阵A  
	cv::solve(X, Y, A, cv::DECOMP_LU);


	auto min_itr = min_element(img_shp.begin(), img_shp.end(), [](const auto& ele1, const auto& ele2)->bool {
		return ele1.y < ele2.y;
		});
	double start_y = min_itr->y;

	auto max_itr = max_element(img_shp.begin(), img_shp.end(), [](const auto& ele1, const auto& ele2)->bool {
		return ele1.y < ele2.y;
		});
	double end_y = max_itr->y;

	for (int i = 0; i < N; i++)
	{
		double y = start_y + (end_y - start_y) / N * i;
		img_shp[i].y = y;
		img_shp[i].x = A.at<double>(0, 0) +
			A.at<double>(1, 0) * y +
			A.at<double>(2, 0) * std::pow(y, 2);/*+
			A.at<double>(3, 0)*std::pow(y, 3)*/
	}

}
#endif