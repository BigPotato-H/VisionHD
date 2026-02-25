// DataIO.cpp : 定义控制台应用程序的入口点。

// for std
// for opencv
#pragma once
#ifndef DataIO_AGV
#define DataIO_AGV

#include "DataIO.h"

class DataIOArgoverse :public DataIO
{
public:
	DataIOArgoverse();
	~DataIOArgoverse();
	size_t getTracePoints(vector<TrajectoryData>& ins_vec);
	size_t getHDLaneDividerInBox(const vector<cv::Point2f>& box, HDObject_VEC& obj_vec, bool keep_inside_intersection = false);
private:
};

#endif