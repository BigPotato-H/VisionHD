#pragma once
#include "CommonHN.h"

#ifndef COM_XMLCONFIG_H
#define COM_XMLCONFIG_H
#include <string>
#include <vector>
#include "tinyxml2.h"
#include <opencv2/core/core.hpp>

//缩短并简化函数调用形式
#define MY_CONFIG HN_GENERAL::my_config

using namespace std;

struct DataPara
{
	string data_name;
	string camera_type;
	vector<double> intrinsic_para;
	vector<double> distort;
	cv::Point3d trans;
	cv::Vec3f rotate;
	vector<cv::Point2f> corners;
	int image_width;
	int image_height;
	cv::Rect image_rect;
	float ego_height;
	string data_path;
	string mid_path;
};

class COMMONHN_API ConfigClass 
{
public:
	string data_path;
	vector<string> sql;
	string mid_path;
	string img_path;
	string data_name;
	DataPara data_para;
	string GPU_id;
	string detection_env;
	string segmentation_env;
	string method;
};

namespace HN_GENERAL{
extern COMMONHN_API ConfigClass my_config;
	
bool COMMONHN_API read_xml_config();

bool read_database(tinyxml2::XMLNode* root, vector<string>& sql);

void read_database(tinyxml2::XMLElement* task, string& sql);

bool read_string(tinyxml2::XMLNode* root, const std::string& flag, std::string& val);

bool read_data_type_paras(tinyxml2::XMLNode* root, const string& data_na, DataPara &type_paras);

void read_data_type_paras(tinyxml2::XMLElement* task, DataPara &type_paras);

}

#endif // !COM_FILE_H


