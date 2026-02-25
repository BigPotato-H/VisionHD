//#pragma once
#ifndef GEOALGORITHM2
#define GEOALGORITHM2

#include <vector>
#include "HNMath/MathType.h"
using namespace std;
using namespace HNMath;

 ///////////////////////////模板函数的实现/////////////////////////////
template<typename T>
double calcPointYaw(const T& p1, const T& p2)
{
	double yaw = 0;

	double diff_x = p2.x - p1.x;
	double diff_y = p2.y - p1.y;

	if (abs(diff_y) < 1E-03)
	{
		yaw = 0;
	}
	else
	{
		//从x轴顺时针转过来的夹角,范围-pi ~pi
		double x_angle = -atan2(diff_y, diff_x);
		yaw = x_angle + M_PI / 2.0;
		if (yaw < 0)
		{
			yaw += M_PI * 2.0;
		}
	}

	return yaw;
}

 //计算折线上某点的航向(以整北方向为参考)
template<typename T>
double calcPolylineYaw(const vector<T>& base_line, int i)
{
	double yaw = 0;
	if (base_line.size() < 2)
	{
		return yaw;
	}
	const T& pt = base_line[i];

	double diff_x = 0;
	double diff_y = 0;
	if (i != base_line.size() - 1)
	{
		diff_x = base_line[i + 1].m_x - pt.m_x;
		diff_y = base_line[i + 1].m_y - pt.m_y;
	}
	else
	{
		diff_x = pt.m_x - base_line[i - 1].m_x;
		diff_y = pt.m_y - base_line[i - 1].m_y;
	}

	if (abs(diff_y) < 1E-03)
	{
		yaw = 0;
	}
	else
	{
		//从x轴顺时针转过来的夹角,范围-pi ~pi
		double x_angle = -atan2(diff_y, diff_x);
		yaw = x_angle + M_PI / 2.0;
		if (yaw < 0)
		{
			yaw += M_PI * 2.0;
		}
	}
	return yaw/*0~2pi*/;
}


template<typename T>
double calcPolylineYaw2(const vector<T>& base_line, int i)
{
	double yaw = 0;
	if (base_line.size() < 2)
	{
		return yaw;
	}
	const T& pt = base_line[i];

	double diff_x = 0;
	double diff_y = 0;
	if (i != base_line.size() - 1)
	{
		diff_x = base_line[i + 1].x - pt.x;
		diff_y = base_line[i + 1].y - pt.y;
	}
	else
	{
		diff_x = pt.x - base_line[i - 1].x;
		diff_y = pt.y - base_line[i - 1].y;
	}

	if (abs(diff_y) < 1E-03)
	{
		yaw = 0;
	}
	else
	{
		//从x轴顺时针转过来的夹角,范围-pi ~pi
		double x_angle = -atan2(diff_y, diff_x);
		yaw = x_angle + M_PI / 2.0;
		if (yaw < 0)
		{
			yaw += M_PI * 2.0;
		}
	}
	return yaw/*0~2pi*/;
}

template<typename T>
T calPointByAltitude(const T& base_point, bool be_left, float heading, float side_dis)
{
	T pt;
	pt.z = base_point.z;
	//有负值
	if (heading > -PII * 2.0 && heading < 0)
	{
		heading = PII * 2.0 + heading;
	}
	//add 20190925 自己计算出的航向角有超过2pi的，需要处理一下
	else if (heading > PII * 2.0)
	{
		heading = -PII * 2.0 + heading;
	}
	//add 20190925 自己计算出的航向角有超过2pi的，需要处理一下 end

	//与正北方向夹角heading
	//与x轴的夹角theta
	float theta = heading;

	if (heading >= 0 && heading <= PII / 2)
	{
		theta = heading;
		if (be_left)
		{
			pt.x = base_point.x - side_dis * cos(theta);
			pt.y = base_point.y + side_dis * sin(theta);
		}
		else
		{
			pt.x = base_point.x + side_dis * cos(theta);
			pt.y = base_point.y - side_dis * sin(theta);
		}
	}

	else 	if (heading > PII / 2 && heading <= PII)
	{
		theta = heading - PII / 2;
		if (be_left)
		{
			pt.x = base_point.x + side_dis * sin(theta);
			pt.y = base_point.y + side_dis * cos(theta);
		}
		else
		{
			pt.x = base_point.x - side_dis * sin(theta);
			pt.y = base_point.y - side_dis * cos(theta);
		}
	}

	else 	if (heading > PII && heading <= PII * 3 / 2)
	{
		theta = heading - PII;
		if (be_left)
		{
			pt.x = base_point.x + side_dis * cos(theta);
			pt.y = base_point.y - side_dis * sin(theta);
		}
		else
		{
			pt.x = base_point.x - side_dis * cos(theta);
			pt.y = base_point.y + side_dis * sin(theta);
		}
	}

	else 	if (heading > PII * 3 / 2 && heading <= PII * 2)
	{
		theta = heading - PII * 3 / 2;
		if (be_left)
		{
			pt.x = base_point.x - side_dis * sin(theta);
			pt.y = base_point.y - side_dis * cos(theta);
		}
		else
		{
			pt.x = base_point.x + side_dis * sin(theta);
			pt.y = base_point.y + side_dis * cos(theta);
		}
	}
	return pt;
}


template<typename T>
T getFootOfPerpendicular(
	const T &pt,     // 直线外一点
	const T &begin,  // 直线开始点
	const T &end)   // 直线结束点
{
	T retVal;

	double dx = begin.x - end.x;
	double dy = begin.y - end.y;
	if (abs(dx) < 0.00000001 && abs(dy) < 0.00000001)
	{
		retVal = begin;
		return retVal;
	}

	double u = (pt.x - begin.x)*(begin.x - end.x) + (pt.y - begin.y)*(begin.y - end.y);
	u = u / ((dx*dx) + (dy*dy));

 	//矫正范围，确保计算的点在线段内
 	if (u < 0)
 	{
 		u = 0;
 	}
 	if (u > 1)
 	{
 		u = 1;
 	}
	retVal.x = begin.x + u*dx;
	retVal.y = begin.y + u*dy;

	return retVal;
}


template<typename T>
float getDistance(const T& p1, const T& p2)
{
	double dist = sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
	return dist;
}

template<typename T>
double getFootOfPerpendicular(const vector<T>& _polyline, const T& _point, T& _nearest_pt)
{
	map< double, T > dist_pt_map;
	transform(_polyline.begin(), _polyline.end() - 1, _polyline.begin() + 1, inserter(dist_pt_map, dist_pt_map.begin()), [_point](const T& _pts, const T& _pte)
		->pair< double, T>
	{
		T nearest_pt = getFootOfPerpendicular(_point, _pts, _pte);
		double dist = sqrt((nearest_pt.x - _point.x) * (nearest_pt.x - _point.x) + (nearest_pt.y - _point.y) * (nearest_pt.y - _point.y));
		return make_pair(dist, nearest_pt);
	});
	_nearest_pt = dist_pt_map.begin()->second;
	return dist_pt_map.begin()->first;
}


//判断点pt在过p1，p2的直线的前进方向的左侧还是右侧。直线的前进方向是从p1到p2.
//在左侧返回1，在右侧返回-1，在直线上返回0.
template<typename T>
int PointAtLineLR(const T &p1, const T &p2, const T &pt)
{
	//double d = (p2.y - p1.y)*pt.x + (p1.x - p2.x)*pt.y + p2.x*p1.y - p1.x*p2.y;
	double d = (p1.y - pt.y)*(p2.x - pt.x) - (p1.x - pt.x)*(p2.y - pt.y);
	if (d < 0)
	{
		return 1;
	}
	else if (d > 0)
	{
		return -1;
	}
	else if (fabs(d) < 1e-6)
	{
		return 0;
	}

	return 0;
}

template<typename T>
void insertPoint(vector<T>& ply, double density)
{
	auto itr_sk = ply.begin();
	auto itr_sk_next = itr_sk;
	for (; itr_sk != ply.end() - 1; itr_sk++)
	{
		itr_sk_next = itr_sk;
		itr_sk_next++;

		if (itr_sk_next == ply.end())
		{
			break;
		}
		double dist = sqrt((itr_sk_next->x - itr_sk->x)*(itr_sk_next->x - itr_sk->x) +
			(itr_sk_next->y - itr_sk->y) * (itr_sk_next->y - itr_sk->y));
		if (dist < density + 1.0)
		{
			continue;
		}
		//内插
		T insert_p;
		insert_p.x = density / dist * (itr_sk_next->x - itr_sk->x) + itr_sk->x;
		insert_p.y = density / dist * (itr_sk_next->y - itr_sk->y) + itr_sk->y;
		itr_sk = ply.insert(itr_sk_next, insert_p);
		itr_sk--;
	}
}

template<typename T>
void insertPointZ(vector<T>& ply, double density)
{
	auto itr_sk = ply.begin();
	auto itr_sk_next = itr_sk;
	for (; itr_sk != ply.end() - 1; itr_sk++)
	{
		itr_sk_next = itr_sk;
		itr_sk_next++;

		if (itr_sk_next == ply.end())
		{
			break;
		}
		double dist = sqrt((itr_sk_next->x - itr_sk->x)*(itr_sk_next->x - itr_sk->x) +
			(itr_sk_next->y - itr_sk->y) * (itr_sk_next->y - itr_sk->y) +
			(itr_sk_next->z - itr_sk->z) * (itr_sk_next->z - itr_sk->z));
		if (dist < density + 1.0)
		{
			continue;
		}
		//内插
		T insert_p;
		insert_p.x = density / dist * (itr_sk_next->x - itr_sk->x) + itr_sk->x;
		insert_p.y = density / dist * (itr_sk_next->y - itr_sk->y) + itr_sk->y;
		insert_p.z = density / dist * (itr_sk_next->z - itr_sk->z) + itr_sk->z;
		itr_sk = ply.insert(itr_sk_next, insert_p);
		itr_sk--;
	}
}

//形点均匀化
template<typename T>
void equalizPolyline(vector<T>& ply, double density)
{
	if (ply.size()  < 2)
	{
		return;
	}
	//计算线长
	double ply_len = 0;
	auto itr_sk = ply.begin() + 1;
	for (; itr_sk != ply.end(); itr_sk++)
	{
		auto itr_sk_last = itr_sk - 1;
		double dist = sqrt((itr_sk_last->x - itr_sk->x)*(itr_sk_last->x - itr_sk->x) +
			(itr_sk_last->y - itr_sk->y) * (itr_sk_last->y - itr_sk->y) + 
			(itr_sk_last->z - itr_sk->z) * (itr_sk_last->z - itr_sk->z));
		ply_len += dist;
	}

	if (ply_len < density ||
		abs(ply_len - density) <= 0.1)
	{
		return;
	}

	vector<T> equ_ply;
	equ_ply.push_back(ply.front());
	int equ_seg_sz = floor(ply_len / density);
	for (int i = 1; i <= equ_seg_sz; i++)
	{
		double seg_len = density * i;
		double sum_len = 0;
		double dist = 0;
		auto itr_p = ply.begin() + 1;
		auto itr_last_p = itr_p - 1;
		for (; itr_p != ply.end(); itr_p++)
		{
			itr_last_p = itr_p - 1;
			dist = sqrt((itr_last_p->x - itr_p->x)*(itr_last_p->x - itr_p->x) +
				(itr_last_p->y - itr_p->y) * (itr_last_p->y - itr_p->y) + 
				(itr_last_p->z - itr_p->z) * (itr_last_p->z - itr_p->z));
			sum_len += dist;
			if (abs(sum_len - seg_len) <= 0.1 ||
				sum_len > seg_len)
			{
				break;
			}
		}
		if (itr_p == ply.end())
		{
			continue;
		}
		T insert_p;
		if (abs(sum_len - seg_len) <= 0.1)
		{
			insert_p = *itr_p;
		}
		else
		{
			double insert_dis = seg_len - (sum_len - dist);
			insert_p.x = insert_dis / dist * (itr_p->x - itr_last_p->x) + itr_last_p->x;
			insert_p.y = insert_dis / dist * (itr_p->y - itr_last_p->y) + itr_last_p->y;
			insert_p.z = insert_dis / dist * (itr_p->z - itr_last_p->z) + itr_last_p->z;
		}
		equ_ply.push_back(insert_p);
	}
	equ_ply.push_back(ply.back());
	ply.swap(equ_ply);

}


//////////////////////////////////////////////////////////////////////////
template <typename PointT> bool
isXYPointIn2DXYPolygon(const PointT &point, const vector<PointT> &polygon)
{
	bool in_poly = false;
	double x1, x2, y1, y2;

	int nr_poly_points = static_cast<int> (polygon.size());
	// start with the last point to make the check last point<->first point the first one
	double xold = polygon[nr_poly_points - 1].x;
	double yold = polygon[nr_poly_points - 1].y;
	for (int i = 0; i < nr_poly_points; i++)
	{
		double xnew = polygon[i].x;
		double ynew = polygon[i].y;
		if (xnew > xold)
		{
			x1 = xold;
			x2 = xnew;
			y1 = yold;
			y2 = ynew;
		}
		else
		{
			x1 = xnew;
			x2 = xold;
			y1 = ynew;
			y2 = yold;
		}

		if ((xnew < point.x) == (point.x <= xold) && (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1))
		{
			in_poly = !in_poly;
		}
		xold = xnew;
		yold = ynew;
	}

	return (in_poly);
}

template <typename PointT>
float getAngle(const vector<PointT>& front_line/*前*/, const vector<PointT>& post_line/*后*/)
{
	if (front_line.size() == 0 ||
		post_line.size() == 0)
	{
		return 90;
	}
	//计算前一条线的尾部
	const PointT& ept = front_line.back();
	PointT to_e_pt;
	KPolylineAlg::getPtAwayfromPolylineEPt(front_line, 4.0, to_e_pt);

	//计算后一条线的起始
	const PointT& spt = post_line.front();
	PointT s_to_pt;
	KPolylineAlg::getPtAwayfromPolylineSPt(post_line, 4.0, s_to_pt);


	float angle = KLineSegmentAlg::calcVectorAngle(to_e_pt, ept, spt, s_to_pt);
	return angle;
}

template <typename PointT>
bool getPolylineInPolygon(const vector<PointT>& _polygon, const vector<PointT>&_line_poly, vector<PointT>& _inside_polyline)
{
	if (_line_poly.size() == 0)
	{
		return false;
	}


	vector<PointT> l_cross_pts;
	KPolylineAlg::calcPolylineCrossingPoint(_line_poly, _polygon, l_cross_pts);
	//无交叉点
	if (l_cross_pts.size() == 0)
	{
		const auto& spt = _line_poly.front();
		const auto& ept = _line_poly.back();
		if (PLANEMATH_CLASS::isPointInside(spt, _polygon) <= 0 ||
			PLANEMATH_CLASS::isPointInside(ept, _polygon) <= 0)
		{
			//整条线在内部
			_inside_polyline = _line_poly;
			return true;
		}
		else
		{
			//在外面
			return false;
		}
	}
	else if (l_cross_pts.size() > 2)//交叉点数大于2
	{
		//外面需要打印log
		return 3;
	}
	//有交叉点
	// 	if (l_cross_pts.size() == 2)
	// 	{
	// 		bool the_same_direction = isPolylineTheSameDirection(l_cross_pts[0], l_cross_pts[1], _line_poly);
	// 		if (!the_same_direction)
	// 		{
	// 			return false;
	// 		}
	// 	}
	//先判断车道线的方向与2个交叉点构成的矢量线的方向是否一致
	//如果不一致，应该是对向的车道线，可以不在这块点云上生成样本

	vector<vector<PointT>> _splitted_polyline;
	KPolylineAlg::cutPolyline(_line_poly, l_cross_pts, _splitted_polyline);


	if (_splitted_polyline.size() == 0)
	{
		return false;
	}

	int idx = -1;
	if (_splitted_polyline.size() == 1)
	{
		idx = 0;
	}
	else if (_splitted_polyline.size() == 3)
	{
		idx = 1;
	}
	else if (_splitted_polyline.size() == 2)
	{
		for (size_t i = 0; i < _splitted_polyline.size(); ++i)
		{
			const auto& ply = _splitted_polyline[i];
			PointT spt = ply.front();
			PointT ept = ply.back();
			spt.m_z = 0;
			ept.m_z = 0;

			if (ply.size() == 2)
			{
				PointT mid_pt;
				mid_pt.x = (spt.x + ept.x) / 2.0;
				mid_pt.y = (spt.y + ept.y) / 2.0;
				if (PLANEMATH_CLASS::isPointInside(mid_pt, _polygon) <= 0)
				{
					idx = i;
					break;
				}
				else
				{
					continue;
				}
			}
			else if (ply.size() > 2)
			{
				PointT mid_pt = ply[1];
				if (PLANEMATH_CLASS::isPointInside(mid_pt, _polygon) <= 0)
				{
					idx = i;
					break;
				}
				else
				{
					continue;
				}
			}
		}
	}

	if (idx == -1)
	{
		return false;
	}

	_inside_polyline = _splitted_polyline[idx];

	return true;
}

template<typename T>
int calcNearestVertex(const vector<T>& _polyline, const T& _pt)
{
	map< double, int> dist_point_map;
	int i = 0;
	transform(_polyline.begin(), _polyline.end(), inserter(dist_point_map, dist_point_map.begin()), [&](const T& _vertex) {
		double dist = sqrt((_pt.x - _vertex.x) * (_pt.x - _vertex.x) + (_pt.y - _vertex.y) * (_pt.y - _vertex.y));
		return make_pair(dist, i++);
	});
	return (dist_point_map.begin())->second;
}

template<typename T>
int PointAtPolylineLR(const vector<T>& ply, const T &pt)
{
	int s_idx = calcNearestVertex(ply, pt);
	if (s_idx == ply.size() - 1)
	{
		s_idx = ply.size() - 2;
	}
	int e_idx = s_idx + 1;

	return PointAtLineLR(ply[s_idx], ply[e_idx], pt);
}

#endif // !GEOALGORITHM2
