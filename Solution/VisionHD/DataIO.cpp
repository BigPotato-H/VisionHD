// DataIO.cpp : 定义控制台应用程序的入口点。
// for std
#include <iostream>
// for opencv
#include "DataIO.h"

#include<fstream>
#include <io.h>
#include "HNMath/GeometricAlgorithm2.h"

#include "HNString/HNString.h"
#include "HNString/EncodeStr.h"
#include "DataManager/WKTCSV.h"
#include "Time/LocalTime.h"
#include "HNFile/File.h"
#include "opencv2/imgproc.hpp"

DataIO::DataIO()
{
}

DataIO::~DataIO()
{
}

void DataIO::initDataPath(const string& root_path)
{
	m_root_path = root_path;
}

void DataIO::initSQLPath(const string& sql_path)
{
	m_sql = sql_path;
}

string DataIO::formatBoxStr(const vector<cv::Point2f>& box, int srid)
{
	if (box.size() != 4)
	{
		return "";
	}

	string box_str = "";
	HNString::FormatA(box_str, "st_geometryfromtext('polygon((%.8f %.8f,%.8f %.8f,%.8f %.8f,%.8f %.8f,%.8f %.8f))', %d)",
		box[0].x, box[0].y,
		box[1].x, box[1].y,
		box[2].x, box[2].y,
		box[3].x, box[3].y,
		box[0].x, box[0].y,
		srid);

	return box_str;
}


void relate2ImageTimestamp(long long tsp)
{
	tm tm_ = HN_GENERAL::stamp_to_standard(tsp);
	char s[100] = { 0 };
	strftime(s, sizeof(s), "%Y-%m-%d %H:%M:%S", &tm_);
}

size_t DataIO::readHDI(const string& file_path, vector<TrajectoryData>& ins_vec)
{
	// 定义输入文件流类对象infile
	ifstream infile(file_path, ios::in);

	if (!infile)
	{  // 判断文件是否存在
		cerr << "open error." << endl;
		return 0;
	}

	char str[255]; // 定义字符数组用来接受读取一行的数据
	TrajectoryData raw;
	while (infile)
	{
		infile.getline(str, 255);  // getline函数可以读取整行并保存在str数组里
		vector<string> str_vec;
//		HNString::SplitA(str, str_vec, "	");
//		HNString::SplitA(str, str_vec, " ");
		HNString::SplitA(str, str_vec, "\t");
		if (str_vec.size() < 16)
		{
			continue;
		}
		raw.name = str_vec[0].c_str();
		raw.point.x = atof(str_vec[9].c_str());

		if (int(raw.point.x) == 0 &&
			int(raw.point.y) == 0)
		{
			continue;
		}
		raw.point.y = atof(str_vec[10].c_str());
		raw.point.z = atof(str_vec[11].c_str());

		raw.lonlat.x = atof(str_vec[12].c_str());
		raw.lonlat.y = atof(str_vec[13].c_str());
		raw.lonlat.z = atof(str_vec[11].c_str());

		raw.heading = atof(str_vec[14].c_str()) / 180.0 * M_PI;
		raw.pitch = atof(str_vec[15].c_str()) / 180.0 * M_PI;
		raw.roll = atof(str_vec[16].c_str()) / 180.0 * M_PI;
		if (raw.heading < 0)
		{
			raw.heading += 2.0 * M_PI;
		}
		ins_vec.push_back(raw);
	}
	infile.close();

	if (ins_vec.size() == 0)
	{
		return 0;
	}
	raw = ins_vec.front();
	
	return ins_vec.size();
}

size_t getFiles(string path, string filter, vector<string>& files)
{
	//文件句柄
	intptr_t   hFile = 0;
	//文件信息
	struct _finddata_t fileinfo;
	string p = path + filter;
	if ((hFile = _findfirst(p.c_str(), &fileinfo)) != -1)
	{
		do
		{
			//如果是目录,迭代之
			//如果不是,加入列表
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(path + fileinfo.name, "", files);
			}
			else
			{
				files.push_back(path + fileinfo.name);
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}

	
	return files.size();
}


void DataIO::saveHDMap(const string& file_path, const HDObject_VEC& obj_vec)
{
	string file_na = file_path + "map.csv";
	ofstream os(file_na, ios::app);
	if (!os.is_open())
	{
		return;
	}
	os.setf(ios::fixed, ios::floatfield);
	os.precision(4);

	for (int i = 0; i < obj_vec.size(); i++)
	{
		const auto& obj = obj_vec[i];
		if (obj.shape.size() < 2)
		{
			continue;
		}
		vector<string> attr_vec;
		attr_vec.push_back(obj.obj_id);
		attr_vec.push_back(to_string(obj.type));
		outAttrCSVFields(os, attr_vec);
		outShpCSVFields(os, obj.shape, 1);
	}
	os.close();
}

void DataIO::saveParas(const string& folder_path, const CamPose& cp)
{
	if (cp.camPose.size() != 6)
	{
		return;
	}

	ofstream of(folder_path, ios::out | ios::app);
	if (!of.is_open())
	{
		return;
	}
	of.setf(ios::fixed, ios::floatfield);
	of.precision(4);
	string line = "";
#if 1
	HNString::FormatA(line, "%s,%d,%.2f,%.8f,%.8f,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
		cp.img_na.c_str(),
		cp.regist_flg,
		cp.regist_probability,
		cp.ins.lonlat.x,
		cp.ins.lonlat.y,
		cp.ins.point.x,
		cp.ins.point.y,
		cp.ins.point.z,
		cp.ins.heading / M_PI * 180.0,
		cp.ins.pitch / M_PI * 180.0,
		cp.ins.roll / M_PI * 180.0,
		cp.camPose[0],
		cp.camPose[1],
		cp.camPose[2],
		cp.camPose[3] / M_PI * 180.0,
		cp.camPose[4] / M_PI * 180.0,
		cp.camPose[5] / M_PI * 180.0);
#else
	HNString::FormatA(line, "%s,%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
		cp.img_na.c_str(),
		cp.regist_probability,
		cp.camPose[0],
		cp.camPose[1],
		cp.camPose[2],
		cp.camPose[3],
		cp.camPose[4],
		cp.camPose[5]);
#endif
	of << line << endl;
	of.close();
}

size_t DataIO::readCamPosePara(const string& folder_path, map<string, CamPose>& pos_map)
{
	ifstream fin(folder_path + "para.csv");
	string line = "";
	while (getline(fin, line))
	{
		vector<string> str_vec;
		HNString::SplitA(line, str_vec, ",");
		if (str_vec.size() == 0)
		{
			continue;
		}
		CamPose cp;
		int seq = 0;
		cp.img_na = str_vec[seq++];

		cp.regist_flg = stoi(str_vec[seq++]);
		cp.regist_probability = stoi(str_vec[seq++]);
		cp.ins.lonlat.x = stod(str_vec[seq++]);
		cp.ins.lonlat.y = stod(str_vec[seq++]);
//		cp.ins.lonlat.z = stod(str_vec[seq++]);
		cp.ins.point.x = stod(str_vec[seq++]);
		cp.ins.point.y = stod(str_vec[seq++]);
		cp.ins.point.z = stod(str_vec[seq++]);
		cp.ins.heading = stod(str_vec[seq++]) / 180.0 * M_PI;
		cp.ins.pitch = stod(str_vec[seq++]) / 180.0 * M_PI;
		cp.ins.roll = stod(str_vec[seq++]) / 180.0 * M_PI;
		cp.camPose.resize(6);
		cp.camPose[0] = stod(str_vec[seq++]);
		cp.camPose[1] = stod(str_vec[seq++]);
		cp.camPose[2] = stod(str_vec[seq++]);
		cp.camPose[3] = stod(str_vec[seq++]) / 180.0 * M_PI;
		cp.camPose[4] = stod(str_vec[seq++]) / 180.0 * M_PI;
		cp.camPose[5] = stod(str_vec[seq++]) / 180.0 * M_PI;
		
		pos_map[cp.img_na] = cp;
	}

	return pos_map.size();
}

void DataIO::getTraceBox(const vector<TrajectoryData>& ins_vec, vector<cv::Point2f>& box, float threshold)
{
	cv::Point2f right_up(-180.0, -90.0);
	cv::Point2f right_down(-180.0, 90.0);
	cv::Point2f left_down(180.0, 90.0);
	cv::Point2f left_up(180.0, -90.0);

	int count = ins_vec.size();
	for (int i = 0; i < count; i++)
	{
		//right_up
		if (right_up.x < ins_vec[i].lonlat.x)
		{
			right_up.x = ins_vec[i].lonlat.x;
		}
		if (right_up.y < ins_vec[i].lonlat.y)
		{
			right_up.y = ins_vec[i].lonlat.y;
		}


		//right_down
		if (right_down.x < ins_vec[i].lonlat.x)
		{
			right_down.x = ins_vec[i].lonlat.x;
		}
		if (right_down.y > ins_vec[i].lonlat.y)
		{
			right_down.y = ins_vec[i].lonlat.y;
		}

		//left_down
		if (left_down.x > ins_vec[i].lonlat.x)
		{
			left_down.x = ins_vec[i].lonlat.x;
		}
		if (left_down.y > ins_vec[i].lonlat.y)
		{
			left_down.y = ins_vec[i].lonlat.y;
		}


		//left_up
		if (left_up.x > ins_vec[i].lonlat.x)
		{
			left_up.x = ins_vec[i].lonlat.x;
		}
		if (left_up.y < ins_vec[i].lonlat.y)
		{
			left_up.y = ins_vec[i].lonlat.y;
		}

	}

	threshold /= 100000;
	right_up.x += threshold;
	right_up.y += threshold;
	right_down.x += threshold;
	right_down.y -= threshold;
	left_down.x -= threshold;
	left_down.y -= threshold;
	left_up.x -= threshold;
	left_up.y += threshold;

	box.push_back(right_up);
	box.push_back(right_down);
	box.push_back(left_down);
	box.push_back(left_up);

}

void DataIO::getTraceXYBox(const vector<TrajectoryData>& ins_vec, vector<cv::Point2f>& box, float threshold)
{
	int count = ins_vec.size();
	if (count == 0)
	{
		return;
	}
	auto init_x = ins_vec[0].point.x;
	auto init_y = ins_vec[0].point.y;
	cv::Point2f right_up(init_x, init_y);
	cv::Point2f right_down(right_up);
	cv::Point2f left_down(right_up);
	cv::Point2f left_up(right_up);

	
	for (int i = 0; i < count; i++)
	{
		//right_up
		if (right_up.x < ins_vec[i].point.x)
		{
			right_up.x = ins_vec[i].point.x;
		}
		if (right_up.y < ins_vec[i].point.y)
		{
			right_up.y = ins_vec[i].point.y;
		}


		//right_down
		if (right_down.x < ins_vec[i].point.x)
		{
			right_down.x = ins_vec[i].point.x;
		}
		if (right_down.y > ins_vec[i].point.y)
		{
			right_down.y = ins_vec[i].point.y;
		}

		//left_down
		if (left_down.x > ins_vec[i].point.x)
		{
			left_down.x = ins_vec[i].point.x;
		}
		if (left_down.y > ins_vec[i].point.y)
		{
			left_down.y = ins_vec[i].point.y;
		}


		//left_up
		if (left_up.x > ins_vec[i].point.x)
		{
			left_up.x = ins_vec[i].point.x;
		}
		if (left_up.y < ins_vec[i].point.y)
		{
			left_up.y = ins_vec[i].point.y;
		}

	}

	right_up.x += threshold;
	right_up.y += threshold;
	right_down.x += threshold;
	right_down.y -= threshold;
	left_down.x -= threshold;
	left_down.y -= threshold;
	left_up.x -= threshold;
	left_up.y += threshold;

	box.push_back(right_up);
	box.push_back(right_down);
	box.push_back(left_down);
	box.push_back(left_up);

}



void DataIO::saveTracePoints(const vector<TrajectoryData>& ins_vec)
{
	ofstream of(m_root_path + "trace_points.csv", ios::out | ios::trunc);
	if (!of.is_open())
	{
		return;
	}
	of.setf(ios::fixed, ios::floatfield);
	of.precision(4);

	for (const auto& ins : ins_vec)
	{
		string line = "";
		HNString::FormatA(line, "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
			ins.name.c_str(),			
			ins.point.x,
			ins.point.y,
			ins.point.z,
			ins.heading / M_PI * 180.0,
			ins.pitch / M_PI * 180.0,
			ins.roll / M_PI * 180.0
			);
		of << line << endl;
	}
	
	of.close();
}


void DataIO::saveJitterCamPose(string file_path, const vector<CamPose>& pose_vec)
{
	ofstream of(file_path, ios::out | ios::trunc);
	if (!of.is_open())
	{
		return;
	}
	of.setf(ios::fixed, ios::floatfield);
	of.precision(4);

	for (const auto& cp : pose_vec)
	{
		string line = "";
		HNString::FormatA(line, "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
			cp.img_na.c_str(),
			cp.camPose[0],
			cp.camPose[1],
			cp.camPose[2],
			cp.camPose[3],
			cp.camPose[4],
			cp.camPose[5]
		);
		of << line << endl;
	}

	of.close();
}

size_t DataIO::readJitterCamPose(string file_path, map<string, CamPose>& pose_map)
{
	ifstream infile(file_path, ios::in);

	if (!infile)
	{  // 判断文件是否存在
		cerr << "open error." << endl;
		return 0;
	}

	string str = "";
	while (getline(infile, str))
	{
		vector<string> str_vec;
		HNString::SplitA(str, str_vec, ",");

		CamPose cp;
		cp.img_na = str_vec[0];
		auto& pose = cp.camPose;
		pose.resize(6);
		for (int i = 0; i < 6; i++)
		{
			pose[i] = stod(str_vec[i +1]);
		}
		pose_map[cp.img_na] = cp;
	}
	infile.close();

	return pose_map.size();
}

void DataIO::saveGTCamPose(string file_path, const vector<CamPose>& pose_vec)
{
	ofstream of(file_path, ios::out | ios::trunc);
	if (!of.is_open())
	{
		return;
	}
	of.setf(ios::fixed, ios::floatfield);
	of.precision(4);

	for (const auto& cp : pose_vec)
	{
		string line = "";
		HNString::FormatA(line, "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
			cp.img_na.c_str(),
			cp.camPose[0],
			cp.camPose[1],
			cp.camPose[2],
			cp.camPose[3],
			cp.camPose[4],
			cp.camPose[5]
		);
		of << line << endl;
	}

	of.close();
}

size_t DataIO::readGTCamPose(string file_path, map<string, CamPose>& pose_map)
{
	ifstream infile(file_path, ios::in);

	if (!infile)
	{  // 判断文件是否存在
		cerr << "open error." << endl;
		return 0;
	}

	string str = "";
	while (getline(infile, str))
	{
		vector<string> str_vec;
		HNString::SplitA(str, str_vec, ",");

		CamPose cp;
		cp.img_na = str_vec[0];
		auto& pose = cp.camPose;
		pose.resize(6);
		for (int i = 0; i < 6; i++)
		{
			pose[i] = stod(str_vec[i + 1]);
		}
		pose_map[cp.img_na] = cp;
	}
	infile.close();

	return pose_map.size();
}

