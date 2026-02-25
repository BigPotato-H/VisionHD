#include "DataIOArgoverse.h"
#include "DataManager/WKTCSV.h"
#include <io.h>
#include <fstream>
#include <iostream>
#include "HNMath/TransRotation.h"
#include <cstdlib>

DataIOArgoverse::DataIOArgoverse()
{
}

DataIOArgoverse::~DataIOArgoverse()
{
}


size_t DataIOArgoverse::getHDLaneDividerInBox(const vector<cv::Point2f>& box, HDObject_VEC& obj_vec,
	bool keep_inside_intersection)
{
	const string& file_na = m_root_path + "hd_map\\ld.csv";
	if (_access(file_na.c_str(), 0) != 0)
	{
		return 0;
	}

	ifstream is(file_na, fstream::in);
	is.setf(ios::fixed, ios::floatfield);
	is.precision(4);

	string line = "";
	while (getline(is, line))
	{
		vector<string> attr_vec;
		if (line.size() == 0)
		{
			continue;
		}
		inAttrCSVFields(line, attr_vec);
		if (attr_vec.size() == 0)
		{
			continue;
		}
		if (!keep_inside_intersection)
		{
			if (attr_vec[2] == "True")
			{
				continue;
			}
		}
		
		vector<cv::Point3d> pt_vec;
		inShpCSVFields(attr_vec.back(), pt_vec, 2);

		HDObject info;
		info.obj_id = to_string(obj_vec.size());
		info.prop_id = attr_vec[0];
		info.type = 13;
		info.shape.swap(pt_vec);

		obj_vec.emplace_back(info);
	}

	is.close();
}



Eigen::Vector3d getEuler(const Eigen::Matrix3d& rr)
{
	Eigen::Vector3d euler = rr.eulerAngles(0, 1, 2);

	// 旋转矩阵 -> 欧拉角(Z-Y-X，即RPY)（确保第一个值的范围在[0, pi]）
	//Eigen::Vector3d euler = rr.eulerAngles(2, 1, 0);


	//旋转矩阵 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
	Eigen::Vector3d eulerAngle_mine;
	Eigen::Matrix3d rot = rr;
	eulerAngle_mine(2) = std::atan2(rot(2, 1), rot(2, 2));
	eulerAngle_mine(1) = std::atan2(-rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
	eulerAngle_mine(0) = std::atan2(rot(1, 0), rot(0, 0));

	euler = eulerAngle_mine;
	//cout << eulerAngle_mine;
	return euler;
}

size_t DataIOArgoverse::getTracePoints(vector<TrajectoryData>& ins_vec)
{
	const string& file_na = m_root_path + "hd_map\\city_2_ego.csv";
	if (_access(file_na.c_str(), 0) != 0)
	{
		return 0;
	}

	ifstream is(file_na, fstream::in);
//	is.setf(ios::fixed, ios::floatfield);
//	is.precision(4);

	string line = "";
	int k = 0;
	while (getline(is, line))
	{
		k++;
		if (k < 200)
		{
		//	continue;
		}
		vector<string> str_vec;
		if (line.size() == 0)
		{
			continue;
		}
		HNString::SplitA(line, str_vec, ":");
		if (str_vec.size() != 2)
		{
			continue;
		}
		TrajectoryData rs;
		rs.name = str_vec[0];

		string aa_str = str_vec[1];
		str_vec.clear();
		HNString::ReplaceA(aa_str, "[", "");
		HNString::ReplaceA(aa_str, "]", "");
		HNString::SplitA(aa_str, str_vec, ",");
		int i = 0;
		Eigen::Matrix4d rt;
		for (const auto& str : str_vec)
		{
			int r = i / 4;
			int c = i - r * 4;
			rt(r, c) = stod(str);

			i++;
		}
		//cout << "---rt:" << endl << rt << endl;

		Eigen::Matrix3d rr = rt.topLeftCorner(3,3);
		Eigen::Vector3d tt = rt.topRightCorner(3,1);
		//cout << "---rr:" << endl << rr << endl;

		//Eigen::Vector3d euler = rr.eulerAngles(0, 1, 2);
		Eigen::Vector3d euler = getEuler(rr);

		Eigen::Matrix3d R_matrix;
		TransRotation::eigenEuler2RotationMatrixd(euler, R_matrix);

		//cout << "---R:" << endl << R_matrix << endl;
		rs.point = cv::Point3d(tt(0), tt(1), tt(2));
				
		rs.roll = euler(1);
		rs.pitch = -euler(2);
		rs.heading = -euler(0) + M_PI / 2.0;

		if (rs.heading > M_PI)
		{
			rs.heading -= M_PI * 2.0;
		}
		if (rs.heading < -M_PI)
		{
			rs.heading += M_PI * 2.0;
		}

	
		if (ins_vec.size() > 0)
		{
			const auto& back = ins_vec.back();
			auto dif = rs.point - back.point;

			double dist = sqrt(dif.ddot(dif));
			if (dist < 1.0)
			{
				continue;
			}			
		}
		//随机抖动

		ins_vec.push_back(rs);
		
	}

	//保存出来
	saveTracePoints(ins_vec);
}
