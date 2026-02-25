//#pragma once
#ifndef WKT_CSV_H
#define WKT_CSV_H

#include <vector>
#include "HNString/HNString.h"

using namespace std;

template<typename T>
void outAttrCSVFields(ofstream& os, const vector</*string*/T>& attr_vec)
{
	if (attr_vec.size() == 0)
	{
		return;
	}

	for (int i = 0; i < attr_vec.size(); i++)
	{
		const auto& p = attr_vec[i];
		//		if (i != attr_vec.size() - 1)
		{
			os << p << ",";
		}
		// 			else
		// 			{
		// 				os << p << endl;
		// 			}
	}
}

template<typename T>
void outShpCSVFields(ofstream& os, const vector<T>& pt_vec, int shp_type)
{
	if (pt_vec.size() == 0)
	{
		//			os << "\"\"" << endl;
		return;
	}
#if 0
	if (pt_vec.size() == 1)
	{
		shp_type = 0;
	}
	else /*if (pt_vec.size() == 2)*/
	{
		shp_type = 1;
	}
	// 		else if (pt_vec.size() >= 3)
	// 		{
	// 			shp_type = 2;
	// 		}
#endif
	//point
	if (shp_type == 0)
	{
		os << "\"SRID=4549;POINT Z(";
	}
	//polyline
	else if (shp_type == 1)
	{
		os << "\"SRID=4549;LINESTRING Z(";
	}
	//polygon
	else if (shp_type == 2)
	{
		os << "\"SRID=4549;POLYGON Z((";
	}
	else if (shp_type == 3)
	{
		os << "\"SRID=4549;MULTIPOINT Z(";
	}
	else
	{
		return;
	}

	for (int i = 0; i < pt_vec.size(); i++)
	{
		const auto& p = pt_vec[i];
		if (i != pt_vec.size() - 1)
		{
	//		os << p.x << " " << p.y << " " << p.z << ",";
			string a = to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z) + ",";
			os << a;
		}
		else
		{
	//		os << p.x << " " << p.y << " " << p.z;
			string a = to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z);
			os << a;
		}
	}
	if (shp_type == 2)
	{
// 		os << ",";
// 		os << pt_vec[0].x << " " << pt_vec[0].y << " " << pt_vec[0].z;

		string a = "," + to_string(pt_vec[0].x) + " " + to_string(pt_vec[0].y) + " " + to_string(pt_vec[0].z);
		os << a;

 		os << "))\"" << endl;
	}
	else
	{
		os << ")\"" << endl;
	}
}

inline void inAttrCSVFields(const string& line_str, vector<string>& attr_vec)
{
	vector<string> mark_strs;
	mark_strs.emplace_back("\"SRID");
	mark_strs.emplace_back("\"LINESTRING");
	mark_strs.emplace_back("\"POYGON");
	mark_strs.emplace_back("\"POINT");
	
	auto e_pos = 0;
	auto  find_mark = mark_strs.begin();
	for (; find_mark != mark_strs.end(); find_mark++)
	{
		string mark = *find_mark;
		e_pos = line_str.find(mark);
		if (e_pos != string::npos)
		{
			break;
		}
	}
	if (e_pos == string::npos)
	{
		return;
	}
	string attr_str = line_str.substr(0, e_pos);
	string shp_str = line_str.substr(e_pos, std::string::npos);
	HNString::SplitA(attr_str, attr_vec, ",");
	/*size_t pos = attr_str.find(",");
	size_t size = attr_str.size();

	while (pos != std::string::npos)
	{
		std::string one_attr = attr_str.substr(0, pos);
		attr_vec.push_back(one_attr);

		attr_str = attr_str.substr(pos + 1, size);
		pos = attr_str.find(",");
	}*/
	attr_vec.push_back(shp_str);
}

template<typename T> 
void inShpCSVFields(std::string shp_str, vector<T>& pt_vec, int shp_type)
{
	getWKTShpZ(shp_str, pt_vec);
	return;
	auto s_pos = shp_str.find("((");
	if (s_pos != std::string::npos)
	{
		shp_str.replace(s_pos, 2, "(");
	}

	auto e_pos = shp_str.find("))");
	if (e_pos != std::string::npos)
	{
		shp_str.replace(e_pos, 2, ")");
	}

	s_pos = shp_str.find("(");
	e_pos = shp_str.rfind(")");
	shp_str = shp_str.substr(s_pos + 1, e_pos - s_pos - 1);

	std::string split_str = ",";
	std::vector<std::string> resVec;

	if ("" == shp_str)
	{
		return;
	}
	/*vector<string> pt_str_vec;
	KString::SplitA(shp_str, pt_str_vec, ",");
	for (const auto& pt_str : pt_vec)
	{
		vector<string> c_vec;
		KString::SplitA(pt_str, c_vec, " ");

	}*/
	//方便截取最后一段数据
	std::string strs = shp_str + split_str;

	size_t pos = strs.find(split_str);
	size_t size = strs.size();

	while (pos != std::string::npos)
	{
		std::string xy_str = strs.substr(0, pos);
		resVec.push_back(xy_str);

		strs = strs.substr(pos + 1, size);
		pos = strs.find(split_str);
	}

	for (int i = 0; i < resVec.size(); i++)
	{
		std::string xy_str = resVec[i];

		size_t pos = xy_str.find(" ");
		size_t pos_y = xy_str.rfind(" ");
		std::string x_str = xy_str.substr(0, pos);
		std::string y_str = xy_str.substr(pos + 1, pos_y - pos);
		std::string z_str = xy_str.substr(pos_y + 1, xy_str.size() - pos_y);

		T p;
		p.x = stod(x_str.c_str());
		p.y = stod(y_str.c_str());
		p.z = stod(z_str.c_str());
		pt_vec.push_back(p);
	}

	//多边形弹掉最后一个点，内存里不需要首尾闭合
	if (shp_type == 2 && pt_vec.size() > 0)
	{
		pt_vec.pop_back();
	}
}

template<typename T>
void getWKTCollectionShpZ(std::string shp_str, vector<T>& pt_vec)
{
	if (shp_str.find("GEOMETRYCOLLECTION") == string::npos)
	{
		return;
	}
	auto s_pos = shp_str.find("(");
	auto e_pos = shp_str.rfind(")");
	shp_str = shp_str.substr(s_pos + 1, e_pos - s_pos - 1);
	vector<string> str_vec;
	HNString::SplitA(shp_str, str_vec, ",P");

	auto itr_shp = str_vec.begin();
	for (; itr_shp != str_vec.end(); itr_shp++)
	{
		vector<T> pp_vec;
		getWKTShpZ(*itr_shp, pp_vec);

		int org_sz = pt_vec.size();
		pt_vec.resize(org_sz + pp_vec.size());
		transform(pp_vec.begin(), pp_vec.end(), pt_vec.begin() + org_sz, [](const auto& p)->T {
			return p; });
	}
	
}
template<typename T>
void getWKTShpZ(std::string shp_str, vector<T>& pt_vec)
{
	auto s_pos = shp_str.rfind("(");
	auto e_pos = shp_str.find(")");

	if (s_pos == std::string::npos ||
		e_pos == std::string::npos)
	{
		return;
	}

	shp_str = shp_str.substr(s_pos + 1, e_pos - s_pos - 1);
	std::string split_str = ",";
	std::vector<std::string> resVec;
	HNString::SplitA(shp_str, resVec, ",");
	if (resVec.size() == 0)
	{
		return;
	}
	for (int i = 0; i < resVec.size(); i++)
	{
		std::string xy_str = resVec[i];
		vector<string> xyz_vec;
		HNString::SplitA(xy_str, xyz_vec, " ");
		T p;
		p.x = stod(xyz_vec[0].c_str());
		p.y = stod(xyz_vec[1].c_str());
		p.z = stod(xyz_vec[2].c_str());
		pt_vec.push_back(p);
	}

	
}

template<typename T>
void getWKTShp(std::string shp_str, vector<T>& pt_vec)
{
	auto s_pos = shp_str.rfind("(");
	auto e_pos = shp_str.find(")");

	if (s_pos == std::string::npos ||
		e_pos == std::string::npos)
	{
		return;
	}

	shp_str = shp_str.substr(s_pos + 1, e_pos - s_pos - 1);
	std::string split_str = ",";
	std::vector<std::string> resVec;

	if ("" == shp_str)
	{
		return;
	}
	//方便截取最后一段数据
	std::string strs = shp_str + split_str;

	size_t pos = strs.find(split_str);
	size_t size = strs.size();

	while (pos != std::string::npos)
	{
		std::string xy_str = strs.substr(0, pos);
		resVec.push_back(xy_str);

		strs = strs.substr(pos + 1, size);
		pos = strs.find(split_str);
	}

	for (int i = 0; i < resVec.size(); i++)
	{
		std::string xy_str = resVec[i];

		size_t pos = xy_str.find(" ");
		std::string x_str = xy_str.substr(0, pos);
		std::string y_str = xy_str.substr(pos + 1, xy_str.size() - pos);

		T p;
		p.x = stod(x_str.c_str());
		p.y = stod(y_str.c_str());
		pt_vec.push_back(p);
	}
}

template<typename T>
string getWKTStr(const vector<T>& pt_vec, int shp_type, int srid = 4326)
{
	std::string shp_str = "";
	if (pt_vec.size() == 0)
	{
		return "";
	}
	//point
	if (shp_type == 0)
	{
		HNString::FormatA(shp_str, "'SRID=%d;POINT Z(", srid);
	}
	//polyline
	else if (shp_type == 1)
	{
		HNString::FormatA(shp_str, "'SRID=%d;LINESTRING Z(", srid);
	}
	//polygon
	else if (shp_type == 2)
	{
		HNString::FormatA(shp_str, "'SRID=%d;POLYGON Z((", srid);
	}
	else if (shp_type == 3)
	{
		HNString::FormatA(shp_str, "'SRID=%d;MULTIPOINT Z(", srid);
	}
	else
	{
		return "";
	}

	string pt_str = "";
	for (int i = 0; i < pt_vec.size(); i++)
	{
		const auto& p = pt_vec[i];
		if (i != pt_vec.size() - 1)
		{
			pt_str += to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z) + ",";
		}
		else
		{
			pt_str += to_string(p.x) + " " + to_string(p.y) + " " + to_string(p.z);
		}
	}
	if (shp_type == 2)
	{
		pt_str += "," + to_string(pt_vec[0].x) + " " + to_string(pt_vec[0].y) + " " + to_string(pt_vec[0].z) + "))'";
	}
	else
	{
		pt_str += ")'";
	}

	if (pt_str == "")
	{
		return "";
	}
	shp_str = shp_str + pt_str;
	return shp_str;
}
#endif // !COMMON
