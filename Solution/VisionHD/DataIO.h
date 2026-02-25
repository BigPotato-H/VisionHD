// DataIO.cpp : 定义控制台应用程序的入口点。

// for std
// for opencv
#pragma once
#ifndef DataIO_H
#define DataIO_H

#include "DataSet.h"

class DataIO
{
public:
	DataIO();
	~DataIO();
	void initDataPath(const string& root_path);
	void initSQLPath(const string& sql_path);
	virtual size_t getHDJunctionInBox(const vector<cv::Point2f>& box, HDObject_VEC& obj_vec) { return 0; };
	virtual size_t getTracePoints(vector<TrajectoryData>& ins_vec) { return 0; };
	virtual size_t getHDLaneDividerInBox(const vector<cv::Point2f>& box, HDObject_VEC& obj_vec, bool keep_inside_intersection = false) { return 0; };

	void saveTracePoints(const vector<TrajectoryData>& ins_vec);

	void saveJitterCamPose(string file_path, const vector<CamPose>& pose_vec);
	size_t readJitterCamPose(string file_path, map<string, CamPose>& pose_map);

	void saveGTCamPose(string file_path, const vector<CamPose>& pose_vec);
	size_t readGTCamPose(string file_path, map<string, CamPose>& pose_map);


	size_t readHDI(const string& file_path, vector<TrajectoryData>& ins_vec);

	void saveHDMap(const string& file_path, const HDObject_VEC& obj_vec);

	void saveParas(const string& folder_path, const CamPose& cp);
	size_t readCamPosePara(const string& folder_path, map<string, CamPose>& pos_map);

	void getConstantVertixes(const string& folder_path, vector<int>& indices);

	void getTraceBox(const vector<TrajectoryData>& ins_vec, vector<cv::Point2f>& box, float threshold);
	void getTraceXYBox(const vector<TrajectoryData>& ins_vec, vector<cv::Point2f>& box, float threshold);
	string formatBoxStr(const vector<cv::Point2f>& box, int srid = 4326);

protected:
	string m_root_path;
	string m_sql;
};



#endif