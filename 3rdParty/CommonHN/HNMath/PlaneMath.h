#ifndef _PLANEMATH_H_
#define _PLANEMATH_H_
#include <math.h>
#include "Coordinate.h"
#include "MathType.h"
#include "Round.h"
#include "Vector2.h"
#include "assert.h"
namespace HNMath {
//公共函数类
template<typename XYType = /*INT32*/double, template<typename T> class PntType = TPoint3d, typename XYSquareType = double>
class PlaneMath
{
public:
	typedef PntType<XYType>					    PointType;					//点
	typedef std::vector<PointType>				PointVec;					//线
	typedef std::vector<PointVec>				PointVecVec;				//multi-line
	typedef KRect<XYType>						RectType;
private:
	PlaneMath(){}
	~PlaneMath(){}

public:
	//////////////////////////////////////////////////////////////////////////
	/// 计算斜率
	/// @return 
	/// 垂直线返回 浮点数最大无效值
	/// 水平线返回0.0	
	static Real calcLineSlope( const PointType& _pt_s, const PointType& _pt_e )
	{
		Real dx = _pt_e.m_x - _pt_s.m_x;
		Real dy = _pt_e.m_y - _pt_s.m_y;

		if( fabs((double)dx) < DOUBLE_EQUAL_LIMIT )
			return DBL_INF;
		else if( fabs((double)dy) < DOUBLE_EQUAL_LIMIT )
		{
			return 0.0;
		}
		else
		{
			return dy / dx;
		}

	}
	//////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////////
	///计算点串的外接矩形
	///@param shp 
	///@returns Garlic::PlaneMath::RectType:
	//////////////////////////////////////////////////////////////////////////////
	static RectType getOuterRect(const PointVec& shp)
	{
		RectType r;
		if (shp.size() > 0)
		{
			r.m_left = shp.front().m_x;
			r.m_bottom = shp.front().m_y;
			r.m_right = shp.front().m_x;
			r.m_top = shp.front().m_y;

			for (size_t i = 0; i < shp.size(); ++i)
			{
				const PointType& p = shp[i];
				r.m_left = p.m_x < r.m_left ? p.m_x : r.m_left;
				r.m_bottom = p.m_y < r.m_bottom ? p.m_y : r.m_bottom;

				r.m_right = p.m_x > r.m_right ? p.m_x : r.m_right;
				r.m_top = p.m_y > r.m_top ? p.m_y : r.m_top;
			}
		}

		return (r);
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算点串的外接矩形
	///@param shp_vec 
	///@returns Garlic::PlaneMath::RectType:
	//////////////////////////////////////////////////////////////////////////////
	static RectType getOuterRect(const PointVecVec& shp_vec)
	{
		RectType r;
		if (shp_vec.size() > 0 && shp_vec.front().size() > 0)
		{
			const PointVec& shp = shp_vec.front();

			r.m_left = shp.front().m_x;
			r.m_bottom = shp.front().m_y;
			r.m_right = shp.front().m_x;
			r.m_top = shp.front().m_y;

			for (size_t j = 0; j < shp_vec.size(); ++j)
			{
				const PointVec& shp = shp_vec[j];
				for (size_t i = 0; i < shp.size(); ++i)
				{
					const PointType& p = shp[i];
					r.m_left = p.m_x < r.m_left ? p.m_x : r.m_left;
					r.m_bottom = p.m_y < r.m_bottom ? p.m_y : r.m_bottom;

					r.m_right = p.m_x > r.m_right ? p.m_x : r.m_right;
					r.m_top = p.m_y > r.m_top ? p.m_y : r.m_top;
				}
			}
		}
	}
	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算两点之间的距离</summary>
	/// <param name="point_a">第1个点</param>
	/// <param name="point_b">第2个点</param>
	/// <returns>第1个点与第2个点之间的距离</returns>
	////////////////////////////////////////////////////////////////////////////////
	static Real getTwoPointsDistance(const PointType& point_a, const PointType& point_b)
	{
		Real dx = (point_a.m_x - point_b.m_x);
		Real dy = (point_a.m_y - point_b.m_y);
		return (sqrt(double(dx * dx + dy * dy)));
	}

	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算点到线之间的距离</summary>
	/// <param name="point">被计算的点</param>
	/// <param name="point_a">线上的1个点</param>
	/// <param name="point_b">线上的第2个点</param>
	/// <returns>参数中第1个点与后2个点所确定的直线的距离</returns>
	////////////////////////////////////////////////////////////////////////////////
	static Real getPointToLineDistance(const PointType& point, const PointType& point_a, const PointType& point_b)
	{
		Real dis = 0.;
		if ((point_a.m_x == point_b.m_x) && (point_a.m_y == point_b.m_y))
		{
			dis = getTwoPointsDistance(point, point_a);
		}
		else
		{
			Real acreage = abs(double(Real(point.m_x - point_a.m_x) * Real(point_a.m_y - point_b.m_y)
				- Real(point.m_y - point_a.m_y) * Real(point_a.m_x - point_b.m_x)));
			dis = acreage / getTwoPointsDistance(point_a, point_b);
		}
		return dis;
	}

	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算点到线段之间的最短距离</summary>
	/// <param name="point">被计算的点</param>
	/// <param name="point_a">线段的1个端点</param>
	/// <param name="point_b">线段的第2个端点</param>
	/// <returns>参数中被计算的点到直线的最短距离（垂线和端点连接线2种情况）</returns>
	////////////////////////////////////////////////////////////////////////////////
	static Real getPointToSegmentShortcut(const PointType& point, const PointType& point_a, const PointType& point_b)
	{
		Real dis = 0.;
		PointType pointX;

		//计算最短距离点的位置
		getClosestPointOnSegment(point_a.m_x,point_a.m_y,point_b.m_x,
			point_b.m_y,point.m_x,point.m_y,pointX.m_x,pointX.m_y);

		dis =  getTwoPointsDistance(point,pointX);

		return dis;
	}

	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算两点确定的射线的弧度</summary>
	/// <param name="coord_from">射线的起点</param>
	/// <param name="coord_to">射线的终点</param>
	/// <param name="radian">计算结果</param>
	/// <returns>射线起点与终点重合返回false,否则为true</returns>
	////////////////////////////////////////////////////////////////////////////////
	static bool getTwoPointsRadian(const PointType& coord_from, const PointType& coord_to, Real& radian)
	{
		radian = 0.0;
		if ((coord_from.m_x == coord_to.m_x) && (coord_from.m_y == coord_to.m_y))
		{
			return(false);
		}
		//此处强制转换是为了兼容无符号类型坐标
		Real dx = Real(INT32(coord_to.m_x) - INT32(coord_from.m_x));
		Real dy = Real(INT32(coord_to.m_y) - INT32(coord_from.m_y));
		Real cos_radian = dx / sqrt(double((dx * dx) + (dy * dy)));
		radian = acos(double(cos_radian));
		if (dy < 0)
		{
			radian = PI * 2.0 - radian;
		}
		return(true);
	}

	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算两线段的夹角</summary>
	/// <param name="first_start_point">第1条线段上的1个点</param>
	/// <param name="first_end_point">第1条线段上的另1个点</param>
	/// <param name="next_start_point">第2条线段上的1个点</param>
	/// <param name="next_start_point">第2条线段上的另1个点</param>
	/// <returns>两线段的夹角</returns>
	////////////////////////////////////////////////////////////////////////////////
	static Real getTwoSegmentsAngle(const  PointType& first_start_point, const  PointType& first_end_point,
		const  PointType& next_start_point, const  PointType& next_end_point)
	{ 
		if ((first_start_point == first_end_point) || (next_start_point == next_end_point))
		{
			return Real(0.0);
		}
		Real vec_x1 = (first_end_point.m_x) - (first_start_point.m_x);
		Real vec_y1 = (first_end_point.m_y) - (first_start_point.m_y);
		Real vec_x2 = (next_end_point.m_x) - (next_start_point.m_x);
		Real vec_y2 = (next_end_point.m_y) - (next_start_point.m_y);
		if (sqrt(double((vec_x1 * vec_x1 + vec_y1 * vec_y1) 
			* (vec_x2 * vec_x2 + vec_y2 * vec_y2))) == 0)
		{
			return 361.;
		}
		Real cos_radian = (vec_x1 * vec_x2 + vec_y1 * vec_y2) /
			sqrt(double((vec_x1 * vec_x1 + vec_y1 * vec_y1) 
			* (vec_x2 * vec_x2 + vec_y2 * vec_y2)));
		//<modified by nas(2418) 2018/9/3 for 修复当两个值很接近时比较错误的问题>
		if (cos_radian >= 1.0)
		{
			cos_radian = 1.0;
		}
		else if (cos_radian <= -1.0)
		{
			cos_radian = -1.0;
		}
		//</ modified by nas(2418) 2018/9/3 for 修复当两个值很接近时比较错误的问题>

		Real radian = acos((double)cos_radian);
		return (radian * 180 / PI);
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算向量v1到v2的角度，逆时针为正
	///@param v1 
	///@param v2 
	///@returns Real:返回值范围为[0~360)
	//////////////////////////////////////////////////////////////////////////////
	static Real getTwoVectorAngle(const Vector2<XYType>& v1, const Vector2<XYType>& v2)
	{
		PointType p0;
        p0.m_x = 0;
        p0.m_y = 0;
		PointType p1;
        p1.m_x = v1.m_x;
        p1.m_y = v1.m_y;
		PointType p2;
        p2.m_x = v2.m_x;
        p2.m_y = v2.m_y;

		//计算两条直线的夹角
		Real deg = getTwoSegmentsAngle(p0, p1, p0, p2);
		if (deg > 360.0)
		{
			//无效值
			return (deg);
		}

		//判断两向量的位置
		Real cv = v1 * v2;
		if (cv > 0)
		{
			//v1在v2的顺时针方向
			return (deg);
		}
		else if (cv < 0)
		{
			//v1在v2的逆时针方向
			return (Real(360.0) - deg);
		}
		else
		{
			//v1和v2共线
			if ( ( Real(p1.m_x) >0 && Real(p2.m_x)  > 0 ) ||
				 ( Real(p1.m_x) <0 && Real(p2.m_x) < 0 ) ) 
			{
				return 0.0;
			} 
			else
			{
				return 180.0;
			}
			/*if (Real(p1.m_x) * Real(p2.m_x) > 0)
			{
			return (0.0);
			}
			else
			{
			return (180.0);
			}*/
		}
	}

	////////////////////////////////////////////////////////////////////////////////
	//// added by lifei 
	//// purpose: 提供两向量V2 到 V1 向量方向
	////  按照V1 的顺时针方向，
	//// @return <0 位于左，否则位于右	
	////////////////////////////////////////////////////////////////////////////////
	static Real getTwoVectorDirection( const PointType& s1, const PointType& e1, const PointType& s2, const PointType& e2)
	{
		Vector2<XYType> v1(e1.m_x - s1.m_x, e1.m_y - s1.m_y);
		Vector2<XYType> v2(e2.m_x - s2.m_x, e2.m_y - s2.m_y);

		return v1*v2;
	

	}
	//////////////////////////////////////////////////////////////////////////////
	///计算向量p1->p2到向量p3->p4的角度
	///@param p1 
	///@param p2 
	///@param p3 
	///@param p4 
	///@returns Real:
	//////////////////////////////////////////////////////////////////////////////
	static Real getTwoVectorAngle(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4)
	{
		Vector2<XYType> v1(p2.m_x - p1.m_x, p2.m_y - p1.m_y);
		Vector2<XYType> v2(p4.m_x - p3.m_x, p4.m_y - p3.m_y);

		return (getTwoVectorAngle(v1, v2));
	}

	//////////////////////////////////////////////////////////////////////////
	/// purpose: 计算点到线段上最短距离的点
	/// @<-_spt:线段起点
	/// @<-_ept:线段尾点
	/// @<-_from_pt 线段外点
	/// @-> _pt 线段上点
	/// @ return : 0，线段上的垂足; 1,线段的起点， 2，线段的尾点 

	//////////////////////////////////////////////////////////////////////////
	static int getClosestPointOnSegment( const PointType& _spt, const PointType& _ept, const PointType& _from_pt, PointType& _pt )
	{
		return getClosestPointOnSegment( _spt.m_x, _spt.m_y, _ept.m_x, _ept.m_y, _from_pt.m_x, _from_pt.m_y, _pt.m_x, _pt.m_y );
	}

	//////////////////////////////////////////////////////////////////////////////
	///点到线段最短连接线与线段的交点
	///@param x1 
	///@param y1 
	///@param x2 
	///@param y2 
	///@param px 
	///@param py 
	///@param nx 
	///@param ny 
	///@returns void:
	//////////////////////////////////////////////////////////////////////////////
	static int getClosestPointOnSegment(const XYType& x1, const XYType& y1,
		const XYType& x2, const XYType& y2,
		const XYType& px, const XYType& py,
		XYType& nx,       XYType& ny)
	{
		Real vx = (x2) - (x1);
		Real vy = (y2) - (y1);
		Real wx = (px) - (x1);
		Real wy = (py) - (y1);

		Real c1 = vx * wx + vy * wy;

		if (c1 <= Real(0.0))
		{
			nx = x1;
			ny = y1;
			return 1;
		}

		Real c2 = vx * vx + vy * vy;

		if (c2 <= c1)
		{
			nx = x2;
			ny = y2;
			return 2;
		}

		Real ratio = c1 / c2;

		nx = x1 + Round<XYType>::get(ratio * vx);
		ny = y1 + Round<XYType>::get(ratio * vy);
		return 0;
	}
	//////////////////////////////////////////////////////////////////////////
	///add by lifei
	///purpose: 计算点到线段上的垂足
	/// @<-_s_pt 线段起点
	/// @<-_e_pt 线段尾点
	/// @<- _pt  发出垂线的点
	/// @->_foot_pt 垂线在线段上的交点
	/// return : true 垂足在线段上/false 垂足不在线段上
	static bool calcDropFootPointOnSegment( const PointType& _s_pt, const PointType& _e_pt, const PointType& _pt, PointType& _foot_pt  )
	{
		// 计算线段的斜率
		Real k1;
		bool inv_k1 = true;
		if ( (_e_pt.m_x - _s_pt.m_x) == 0  )
		{
			k1 = 0;
		}
		else if ( _e_pt.m_y - _s_pt.m_y  == 0 )
		{			
			inv_k1 = false;
		}
		else
		{
			k1 = ( _e_pt.m_y - _s_pt.m_y )/ (_e_pt.m_x - _s_pt.m_x);
			k1 = 1.0 /(double) k1 ;
		}

		PointType toward_pt;

		// 计算垂直线上另外一个点
		// 设定另外一个点的X坐标与_s_pt 一致
		if ( inv_k1 )
		{
			toward_pt.m_x = _s_pt.m_x;
			toward_pt.m_y  = k1 * ( _pt.m_x -  _s_pt.m_x ) + _pt.m_y;
		}
		else
		{
			toward_pt.m_x = _pt.m_x;
			toward_pt.m_y = _s_pt.m_y;
		}
		//计算射线交点
		if ( getRayLinesegIntersection( _pt, toward_pt, _s_pt, _e_pt, _foot_pt ) || getRayLinesegIntersection( toward_pt, _pt, _s_pt, _e_pt, _foot_pt ))
		{
			return true;
		}
		else
			return false;
		


	}
	////////////////////////////////////////////////////////////////////////

	//////////////////////////////////////////////////////////////////////////
	/// added byl lifei 
	/// purpose :求点到线段垂线在线段上的交点
	/// @<-_s_pt 线段起点
	/// @<-_e_pt 线段尾点
	/// @<- _pt  发出垂线的点
	/// @->_foot_pt 垂线在线段上的交点
	/// return : true 有交点，false 无交点
	/////////////////////////////////////////////////////////////////////////
	static  bool getVerticalFootPointOnSegment( const PointType& _s_pt, const PointType& _e_pt, const PointType& _pt, PointType& _foot_pt  )
	{

		PointType foot_pt;

		getClosestPointOnSegment( _s_pt.m_x, _s_pt.m_y, _e_pt.m_x, _e_pt.m_y, _pt.m_x, _pt.m_y, foot_pt.m_x, foot_pt.m_y );

		if ( ( foot_pt.m_x == _s_pt.m_x && foot_pt.m_y == _s_pt.m_y) || ( foot_pt.m_x == _e_pt.m_x && foot_pt.m_y == _e_pt.m_y) )
		{
			_foot_pt = foot_pt;
			//严格来说，还需要进行角度判定
			// 距离判定
			XYType dist1 =getTwoPointsDistance( _pt, foot_pt );
			XYType dist2 =getPointToLineDistance(  _s_pt, _e_pt, _pt );
			if ( dist1 > dist2 )
			{
				return false;
			}
			else
			{
				return true;
			}			
			
		}
		else
		{
			_foot_pt = foot_pt;
			return true;
		}


	}

	//////////////////////////////////////////////////////////////////////////////
	///计算直线交点，平行或重合则返回false，相交返回true，并返回交点P
	///@param p1 
	///@param p2 
	///@param p3 
	///@param p4 
	///@param p 
	///@returns bool:
	//////////////////////////////////////////////////////////////////////////////
	static bool getLineIntersectPoint(const PointType& p1, const PointType& p2, 
								const PointType& p3, const PointType& p4,
								PointType& p)
	{
		// 计算是否平行或重合
		Real slop1 = calcLineSlope( p1, p2 );
		Real slop2 = calcLineSlope( p3, p4 );
		// 
		// 判定slope是否为垂直平行
		if (!validateDBL( slop1) && !validateDBL( slop2 ) )
		{
			// 垂线的斜率是无穷大
			// 两条线都是垂直平行
			return false;
		}
		else if ( slop2 == slop1 )
		{
			// 平行
			return false;
		}

		
		
		Real x1 = (p1.m_x);
		Real y1 = (p1.m_y);
		Real x2 = (p2.m_x);
		Real y2 = (p2.m_y);
		Real x3 = (p3.m_x);
		Real y3 = (p3.m_y);
		Real x4 = (p4.m_x);
		Real y4 = (p4.m_y);

		Real k1 = 0.0;
		Real k2 = 0.0;
		Real px = 0.0;
		Real py = 0.0;
		Real dy1 = y2 - y1;
		Real dy2 = y4 - y3;
		Real dx1 = x2 - x1;
		Real dx2 = x4 - x3;
#if 0 
		////if ((y2 - y1) * (x4 - x3) == (y4 -y3) * (x2 - x1))
		////Real dd = dy1 * dx2 / dy2 * dx1;
		//if (dy1 * dx2 == dy2 * dx1 || dy1 / dx1 == dy2 / dx2 )
		//{
		//	//平行或者重合
		//	return (false);
		//}
#endif
		if (x1 == x2)
		{
			k2 = (y4 - y3) / (x4 - x3);
			px = x1;
			py = k2 * (x1 - x3) + y3;
		}
		else if (x3 == x4)
		{
			k1 = (y2 - y1) / (x2 - x1);
			px = x3;
			py = k1 * (x3 - x1) + y1;
		}
		else
		{
			//常规计算
			k1 = (y2 - y1) / (x2 - x1);
			k2 = (y4 - y3) / (x4 - x3);
			px = (k1 * x1 - k2 * x3 + y3 - y1) / (k1 - k2);
			py = k1 * (px - x1) + y1;
		}
		
		p.m_x = Round<XYType>::get(px);
		p.m_y = Round<XYType>::get(py);

		return (true);
	}

	//////////////////////////////////////////////////////////////////////////////
	///两线段交点
	///@param x1 
	///@param y1 
	///@param x2 
	///@param y2 
	///@param x3 
	///@param y3 
	///@param x4 
	///@param y4 
	///@param ix 
	///@param iy 
	///@returns bool: 
	//////////////////////////////////////////////////////////////////////////////
	static bool getSegmentIntersectPoint(const XYType& px1, const XYType& py1,
		const XYType& px2, const XYType& py2,
		const XYType& px3, const XYType& py3,
		const XYType& px4, const XYType& py4,
		XYType& ix,       XYType& iy)
	{
		Real x1 = px1;
		Real y1 = py1;
		Real x2 = px2;
		Real y2 = py2;
		Real x3 = px3;
		Real y3 = py3;
		Real x4 = px4;
		Real y4 = py4;

		Real ax = x2 - x1;
		Real bx = x3 - x4;

		Real lowerx;
		Real upperx;
		Real uppery;
		Real lowery;

		if (ax < Real(0.0))
		{
			lowerx = x2;
			upperx = x1;
		}
		else
		{
			upperx = x2;
			lowerx = x1;
		}

		if (bx > Real(0.0))
		{
			if ((upperx < x4) || (x3 < lowerx))
				return false;
		}
		else if ((upperx < x3) || (x4 < lowerx))
			return false;

		Real ay = y2 - y1;
		Real by = y3 - y4;

		if (ay < Real(0.0))
		{
			lowery = y2;
			uppery = y1;
		}
		else
		{
			uppery = y2;
			lowery = y1;
		}

		if (by > Real(0.0))
		{
			if ((uppery < y4) || (y3 < lowery))
				return false;
		}
		else if ((uppery < y3) || (y4 < lowery))
			return false;

		Real cx = x1 - x3;
		Real cy = y1 - y3;
		Real d  = (by * cx) - (bx * cy);
		Real f  = (ay * bx) - (ax * by);

		if (f > Real(0.0))
		{
			if ((d < Real(0.0)) || (d > f))
				return false;
		}
		else if ((d > Real(0.0)) || (d < f))
			return false;

		Real e = (ax * cy) - (ay * cx);

		if (f > Real(0.0))
		{
			if ((e < Real(0.0)) || (e > f))
				return false;
		}
		else if ((e > Real(0.0)) || (e < f))
			return false;

		Real ratio = (ax * -by) - (ay * -bx);

		if (ratio  != 0.0)
		{
			ratio = ((cy * -bx) - (cx * -by)) / ratio;
			ix    = x1 + Round<XYType>::get(ratio * ax);
			iy    = y1 + Round<XYType>::get(ratio * ay);
		}
		else
		{
			if ((ax * -cy) == (-cx * ay))
			{
				ix = Round<XYType>::get(x3);
				iy = Round<XYType>::get(y3);
			}
			else
			{
				ix = Round<XYType>::get(x4);
				iy = Round<XYType>::get(y4);
			}
		}

		return true;
	}

	//////////////////////////////////////////////////////////////////////////////
	///获取线段的交点
	///@param p1 
	///@param p2 
	///@param p3 
	///@param p4 
	///@param ip 
	///@returns bool: 两线段有相交，返回true，否则false
	//////////////////////////////////////////////////////////////////////////////
	static bool getSegmentIntersectPoint(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4, 
		PointType& ip)
	{
		return (getSegmentIntersectPoint(p1.m_x, p1.m_y,
			p2.m_x, p2.m_y,
			p3.m_x, p3.m_y,
			p4.m_x, p4.m_y,
			ip.m_x, ip.m_y));
	}

	//////////////////////////////////////////////////////////////////////////////
	///取得从点P1沿线段P1-P2距离为d的点
	///@param p1	点p1
	///@param p2	点p2
	///@param d		距离d
	///@returns PointType:
	//////////////////////////////////////////////////////////////////////////////
	static PointType getDistancedPoint(const PointType& p1, const PointType& p2, XYType d)
	{
		if (p1.m_x == p2.m_x)
		{
			//竖直线段
			PointType p(p1);
			if (p2.m_y > p1.m_y)
			{
				p.m_y += d;
			}
			else
			{
				p.m_y -= d;
			}
			return (p);
		}
		else
		{
			Real k = Real(p2.m_y - p1.m_y) / (p2.m_x - p1.m_x);
			PointType p(p1);
			Real tmp_val = sqrt(1.0 / (1.0 + double(k * k))) * d;
			if (p2.m_x > p1.m_x)
			{
				//按照X的增量方向进行递增
				tmp_val = tmp_val;
			}
			else
			{
				tmp_val = -tmp_val;
			}

			p.m_x += Round<XYType>::get(tmp_val);
			p.m_y += Round<XYType>::get(tmp_val * k);

			return (p);
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///在线段P1-P2上，从P1开始按距离d，等距离取得分割点，并保持第一个分割点距离P1
	///和最后1个分割点距离P2相同
	///@param p1 起点
	///@param p2 终点
	///@param d 分割距离
	///@param pnt_vec 分割点的集合
	///@param b_include_begin_end 是否包括P1和P2在内
	///@returns void:
	//////////////////////////////////////////////////////////////////////////////
	static void getDividedPoints(const PointType& p1, const PointType& p2, 
		const XYType& d, PointVec& pnt_vec, bool b_include_begin_end = false)
	{
		Real len = getTwoPointsDistance(p1, p2);
		if (len < d)
		{
			return;
		}

		//线段长度和d长度相等
		if (len == d)
		{
			if (b_include_begin_end)
			{
				pnt_vec.push_back(p1);
				pnt_vec.push_back(p2);
			}

			return;
		}
		
		//线段长度>d
		Real delta_l = (len - Round<INT32, ERM_TRUNC>::get(len / d) * d) / 2.0;

		if (b_include_begin_end)
		{
			pnt_vec.push_back(p1);
		}

		if (delta_l == 0)
		{
			//确定还要加入多少个点
			PointType p = p1;
			INT32 pnt_count = Round<INT32, ERM_TRUNC>::get(len / d) - 1;
			for (INT32 i = 1; i <= pnt_count; ++i)
			{
				p = getDistancedPoint(p, p2, d);
				pnt_vec.push_back(p);
			}
		}
		else
		{
			//先放入第一个点
			PointType p = getDistancedPoint(p1, p2, Round<XYType>::get(delta_l));
			pnt_vec.push_back(p);

			//确定还要加入多少个点
			INT32 pnt_count = Round<INT32, ERM_TRUNC>::get(len / d);
			for (INT32 i = 1; i <= pnt_count; ++i)
			{
				p = getDistancedPoint(p, p2, d);
				pnt_vec.push_back(p);
			}
		}

		if (b_include_begin_end)
		{
			pnt_vec.push_back(p2);
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///在折线段line上，从起点开始按距离d等距离取分割点，并保持第一个分割点距离起点
	///和最后1个分割点距离终点相同。该方法只关注取得的分割点坐标，而没有给出分割点
	///在原始折线段上的位置。
	///@param line 
	///@param d 
	///@param pnt_vec 
	///@param b_include_begin_end 
	///@returns void:
	//////////////////////////////////////////////////////////////////////////////
	static void getDividedPoints(PointVec& line, const XYType& d, 
		PointVec& pnt_vec, bool b_include_begin_end = false)
	{
		Real dis = d;
		Real len = 0.0;
		vector<Real> len_vec;

		Real sum_len = 0;
		//先把折线段上的每段距离都算出来
		len_vec.reserve(line.size() - 1);
		for (size_t i = 0; i < line.size() - 1; ++i)
		{
			PointType& p1 = line[i];
			PointType& p2 = line[i + 1];
			len_vec.push_back(getTwoPointsDistance(p1, p2));
			sum_len += len_vec[i];
		}
		//起始位置
		Real delta_len = (sum_len - Round<INT32, ERM_TRUNC>::get(sum_len / dis) * dis) / 2.0;

		if (b_include_begin_end)
		{
			pnt_vec.push_back(line.front());
		}
		//遍历line取点
		dis = (delta_len > 0) ? delta_len : dis;
		for (size_t i = 0; i < line.size() - 1; ++i)
		{
			PointType p1 = line[i];
			PointType p2 = line[i + 1];
			len = len_vec[i];

			while (len >= dis)
			{
				p1 = getDistancedPoint(p1, p2, Round<XYType>::get(dis));
				pnt_vec.push_back(p1);

				len -= dis;
				if (!(fabs(dis - d) < DBL_EPSILON))
				{
					//由于dis可能是上条线段处理过的，不是原始
					//长度，所以第一次处理过后要还原
					dis = d;
				}
			}

			//从dis中去掉剩下的一段len，或者当前线段len比dis小
			//直接跳过这段，不能有点存在
			dis -= len;
		}
		if (b_include_begin_end)
		{
			pnt_vec.push_back(line.back());
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///获取线段和面的所有交点，方法要求面的首尾点坐标相同
	///@param face 
	///@param p1 
	///@param p2 
	///@param intersection_vec 
	///@returns void:
	//////////////////////////////////////////////////////////////////////////////
	static void getSegmentAndFaceIntersections(const PointVecVec& face, const PointType& p1, 
		const PointType& p2, PointVec& intersection_vec)
	{
		for (size_t i = 0; i < face.size(); ++i)
		{
			const PointVec& shp = face[i];

			if (shp.size() < 2)
			{
				continue;
			}

			for (size_t j = 0; j < shp.size() - 1; ++j)
			{
				const PointType& p_j = shp[j];
				const PointType& p_j_n = shp[j + 1];
				PointType inter;

				bool b_inter = getSegmentIntersectPoint(p1, p2, p_j, p_j_n, inter);

				if (b_inter)
				{
					PointType p;
					p.m_x = inter.m_x;
					p.m_y = inter.m_y;
					intersection_vec.push_back(p);
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///判断点p是否在p1-p2线段上，是则返回true，否则返回false，端点也判定为线上
	///@param p 
	///@param p1 
	///@param p2 
	///@returns bool: 
	//////////////////////////////////////////////////////////////////////////////
	static bool isPointOnSegment(const PointType& p, const PointType& p1, const PointType& p2)
	{
        XYSquareType d1 = getTwoPointsDistance(p, p1);
        XYSquareType d2 = getTwoPointsDistance(p, p2);
        XYSquareType d  = getTwoPointsDistance(p1, p2);
		if (d1 + d2 == d)
		{
				return (true);
		}
		else
		{
			return (false);
		}
	}
	//////////////////////////////////////////////////////////////////////////////
	///判断点是否在face内，是则返回true，否则返回false，点在多边形的边上也算作内部
	///@param p
	///@param face 
	///@returns int: >0表示在多边形外，=0表示在多边形边上，<0表示在多边形内
	//////////////////////////////////////////////////////////////////////////////
	static int isPointInside(const PointType& p, const PointVec& face)
	{
		UINT32 int_count = 0;
		for (size_t i = 0; i < face.size() - 1; ++i)
		{
			const PointType& p1 = face[i];
			const PointType& p2 = face[i + 1];

			//做水平射线
			PointType p3 = p;
			XYType max_x = max(p1.m_x, p2.m_x);
			p3.m_x = (max_x > p.m_x ? max_x : p.m_x) + 1;

			if (isPointOnSegment(p, p1, p2))
			{
				return (0);
			}

			if (p1.m_y != p2.m_y)
			{
				//判断Y坐标较大的是否在p-p3上，或者p-p3和p1-p2相交
				PointType p_y_max = (p1.m_y > p2.m_y) ? p1 : p2;
				PointType p_y_min = (p1.m_y > p2.m_y) ? p2 : p1;
// 				if (isPointOnSegment(p_y_max, p, p3))
// 				{
// 					int_count++;
// 				}
				//只要水平射线和当前线段有交点，并且交点不是Y坐标较小的点就OK
				if (isSegmentIntersect(p, p3, p1, p2) && !isPointOnSegment(p_y_min, p, p3))
				{
					int_count++;
				}
// 				else
// 				{
// 					//nothing
// 				}
			}
		}

		return (int_count & 1 ? -1 : 1);
	}
	//////////////////////////////////////////////////////////////////////////////
	///判断点是否在复杂多边形内
	///@param p	要判断的点
	///@param face_vec 复杂多边形的边界
	///@returns int: >0表示在多边形外，=0表示在多边形边上，<0表示在多边形内
	//////////////////////////////////////////////////////////////////////////////
	static int isPointInside(const PointType& p, const PointVecVec& face_vec)
	{
		UINT32 int_count = 0;
		for (size_t f = 0; f < face_vec.size(); ++f)
		{
			const PointVec& face = face_vec.at(f);
			for (size_t i = 0; i < face.size() - 1; ++i)
			{
				const PointType& p1 = face[i];
				const PointType& p2 = face[i + 1];

				//做水平射线
				PointType p3 = p;
				XYType max_x = max(p1.m_x, p2.m_x);
				p3.m_x = (max_x > p.m_x ? max_x : p.m_x) + 1;

				if (isPointOnSegment(p, p1, p2))
				{
					return (0);
				}

				if (p1.m_y != p2.m_y)
				{
					//判断Y坐标较大的是否在p-p3上，或者p-p3和p1-p2相交
					PointType p_y_max = (p1.m_y > p2.m_y) ? p1 : p2;
					PointType p_y_min = (p1.m_y > p2.m_y) ? p2 : p1;
					//只要水平射线和当前线段有交点，并且交点不是Y坐标较小的点就OK
					if (isSegmentIntersect(p, p3, p1, p2) && !isPointOnSegment(p_y_min, p, p3))
					{
						int_count++;
					}
				}
			}
		}

		return (int_count & 1 ? -1 : 1);
	}
	//////////////////////////////////////////////////////////////////////////////
	///判断线段是否在face内，是则返回true，否则返回false
	///@param p1 
	///@param p2 
	///@param face 
	///@returns bool:
	//////////////////////////////////////////////////////////////////////////////
	static bool isSegmentInside(const PointType& p1, const PointType& p2, const PointVec& face)
	{
 		if (isPointInside(p1, face) < 0 || isPointInside(p2, face) < 0)
 		{
 			return (false);
 		}

		PointVec pnt_vec;
		for (size_t i = 0; i < face.size() - 1; ++i)
		{
			const PointType& p_a = face[i];
			const PointType& p_b = face[i + 1];

			bool b_int_on_line = false;
			if (isPointOnSegment(p1, p_a, p_b))
			{
				pnt_vec.push_back(p1);
				b_int_on_line = true;
			}
			else if (isPointOnSegment(p2, p_a, p_b))
			{
				pnt_vec.push_back(p2);
				b_int_on_line = true;
			}
			else if (isPointOnSegment(p_a, p1, p2))
			{
				pnt_vec.push_back(p_a);
				b_int_on_line = true;
			}
			else if (isPointOnSegment(p_b, p1, p2))
			{
				pnt_vec.push_back(p_b);
				b_int_on_line = true;
			}

			if (!b_int_on_line && isSegmentIntersect(p1, p2, p_a, p_b))
			{
				return (false);
			}
		}

		if (pnt_vec.size() == 0)
		{
			return (true);
		}
		else
		{
			std::sort(pnt_vec.begin(), pnt_vec.end());
			for (size_t i = 0; i < pnt_vec.size() - 1; ++i)
			{
				PointType& pa = pnt_vec[i];
				PointType& pb = pnt_vec[i + 1];
				PointType p_c;
				p_c.m_x = Round<XYType>::get(pa.m_x / 2.0 + pb.m_x / 2.0);
				p_c.m_y = Round<XYType>::get(pa.m_y / 2.0 + pb.m_y / 2.0);

				if (!isPointInside(p_c, face))
				{
					return (false);
				}
			}

			return (true);
		}
	}
	//////////////////////////////////////////////////////////////////////////////
	///测试face1是否在face2内，如果face1在face2内，返回true，否则返回false
	///@param face1 
	///@param face2 
	///@returns bool:
	//////////////////////////////////////////////////////////////////////////////
	static bool isFaceInside(const PointVec& face1, const PointVec& face2)
	{
		for (size_t i = 0; i < face1.size() - 1; ++i)
		{
			const PointType& p1 = face1[i];
			const PointType& p2 = face1[i + 1];

			if (!isSegmentInside(p1, p2, face2))
			{
				return (false);
			}
		}

		return (true);
	}
	//////////////////////////////////////////////////////////////////////////////
	///计算面积
	///@param polygon 
	///@returns XYType:
	//////////////////////////////////////////////////////////////////////////////
	static XYSquareType getArea(const PointVec& polygon, bool is_sign = false)
	{
		if (polygon.size() < 3) 
		{
			return XYSquareType(0);
		}
		Real result = Real(0.0);
		std::size_t j = polygon.size() - 1;
		for(std::size_t i = 0; i < polygon.size(); ++i)
		{
			result += ((polygon[j].m_x * polygon[i].m_y) - (polygon[j].m_y * polygon[i].m_x));
			j = i;
		}
		result = result / 2;
		return (is_sign) 
			? Round<XYSquareType>::get(result) 
			: Round<XYSquareType>::get(fabs(double(result)));
	}

	//////////////////////////////////////////////////////////////////////////////
	///判断两线段是否相交，顶点相同也算作相交
	///@param x1 
	///@param y1 
	///@param x2 
	///@param y2 
	///@param x3 
	///@param y3 
	///@param x4 
	///@param y4 
	///@returns bool:
	//////////////////////////////////////////////////////////////////////////////
	static bool isSegmentIntersect(const XYType& x1, const XYType& y1,
		const XYType& x2, const XYType& y2,
		const XYType& x3, const XYType& y3,
		const XYType& x4, const XYType& y4)
	{
		//快速排斥测试
		KRect<XYType> r1;
		r1.m_left = x1 < x2 ? x1 : x2;
		r1.m_bottom = y1 < y2 ? y1 : y2;
		r1.m_right = x1 > x2 ? x1 : x2;
		r1.m_top = y1 > y2 ? y1 : y2;

		KRect<XYType> r2;
		r2.m_left = x3 < x4 ? x3 : x4;
		r2.m_bottom = y3 < y4 ? y3 : y4;
		r2.m_right = x3 > x4 ? x3 : x4;
		r2.m_top = y3 > y4 ? y3 : y4;

		if ((r1.m_right < r2.m_left) || (r1.m_top < r2.m_bottom) 
			|| (r1.m_left > r2.m_right) || (r1.m_bottom > r2.m_top))
		{
			return (false);
		}
		//跨立试验
		Real rx1 = x1;
		Real ry1 = y1;
		Real rx2 = x2;
		Real ry2 = y2;
		Real rx3 = x3;
		Real ry3 = y3;
		Real rx4 = x4;
		Real ry4 = y4;

		Real v1_x = rx1 - rx3;
		Real v1_y = ry1 - ry3;
		Real v2_x = rx4 - rx3;
		Real v2_y = ry4 - ry3;
		Real v3_x = rx2 - rx3;
		Real v3_y = ry2 - ry3;

		if ((v1_x * v2_y - v2_x * v1_y) * (v3_x * v2_y - v2_x * v3_y) <= 0)
		{
			return (true);
		}
		else
		{
			return (false);
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///判断2线段是否相交
	///@param p1 
	///@param p2 
	///@param p3 
	///@param p4 
	///@returns bool:
	//////////////////////////////////////////////////////////////////////////////
	static bool isSegmentIntersect(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4)
	{
		return (isSegmentIntersect(p1.m_x, p1.m_y, p2.m_x, p2.m_y,
			p3.m_x, p3.m_y, p4.m_x, p4.m_y));
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算地球球面距离算法（返回以米为单位）
	///@param p1 
	///@param p2 
	///@returns Real:
	//////////////////////////////////////////////////////////////////////////////
	static Real calcSphericalDistance(const PointType& p1, const PointType& p2)
	{
		if (p1 == p2)
		{
			return Real(0.0);
		}

		Real lat1;
		Real lon1;
		Real lat2;
		Real lon2;

		lat1 = DE2RA * (Real)p1.m_y;
		lon1 = -DE2RA * (Real)p1.m_x;
		lat2 = DE2RA * (Real)p2.m_y;
		lon2 = -DE2RA *(Real)p2.m_x;

		Real F = (lat1 + lat2) / 2.0;
		Real G = (lat1 - lat2) / 2.0;
		Real L = (lon1 - lon2) / 2.0;

		Real sing = sin(double(G));//sine(G);//
		Real cosl = cos(double(L));//cosine(L);//
		Real cosf = cos(double(F));//cosine(F);//
		Real sinl = sin(double(L));//sine(L);//
		Real sinf = sin(double(F));//sine(F);//
		Real cosg = cos(double(G));//cosine(G);//

		Real S = sing*sing*cosl*cosl + cosf*cosf*sinl*sinl;
		Real C = cosg*cosg*cosl*cosl + sinf*sinf*sinl*sinl;
		Real W = atan2(sqrt(double(S)),sqrt(double(C)));
		Real R = Real(sqrt(double(S*C)))/W;
		Real H1 = (Real(3.0) * R - 1.0) / (Real(2.0) * C);
		Real H2 = (Real(3.0) * R + 1.0) / (Real(2.0) * S);
		Real D = Real(2.0) * W * ERAD;

		return (Real)(Real(1.0) * 1000 * D * (Real(1) + FLATTENING * H1 * sinf*sinf*cosg*cosg -
			FLATTENING*H2*cosf*cosf*sing*sing));
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算地球球面距离（返回以米为单位）
	///@param x1 
	///@param y1 
	///@param x2 
	///@param y2 
	///@returns Real:
	//////////////////////////////////////////////////////////////////////////////
	static Real calcSphericalDistance(const XYType& x1, const XYType& y1,
		const XYType& x2, const XYType& y2)
	{
		PointType p1(x1, y1);
		PointType p2(x2, y2);

		return (calcSphericalDistance(p1, p2));
	}

	//////////////////////////////////////////////////////////////////////////////
	///判断简单多边形是否是逆时针方向
	///@param shp 简单多边形，无三点共线，无自交叉 
	///@returns bool: true表示逆时针方向，false表示顺时针方向
	//////////////////////////////////////////////////////////////////////////////
	static bool isCounterClock(const PointVec& shp)
	{
		if (shp.size() < 3)
		{
			return (false);
		}
		size_t prev_idx = shp.back() != shp.front() ? shp.size() - 1 : shp.size() - 2;
		size_t cur_idx = shp.back() != shp.front() ? 0 : 1;
		size_t next_idx = cur_idx + 1;

		//求Y方向极值点
		XYType max_y = shp.front().m_y;
		for (size_t i = cur_idx; i < shp.size(); ++i)
		{
			if (shp[i].m_y > max_y)
			{
				max_y = shp[i].m_y;
				cur_idx = i;
				prev_idx = cur_idx - 1;
				if (i == shp.size() - 1)
				{
					next_idx = 0;
				}
				else
				{
					next_idx = cur_idx + 1;
				}
			}
		}

		//计算极值点前后向量的方向，如果是逆时针，则多边形为逆时针方向，否则就是顺时针
		Vector2<XYType> v1(shp[cur_idx].m_x - shp[prev_idx].m_x,
			shp[cur_idx].m_y - shp[prev_idx].m_y);
		Vector2<XYType> v2(shp[next_idx].m_x - shp[cur_idx].m_x,
			shp[next_idx].m_y - shp[cur_idx].m_y);

		return (v1 * v2 > 0);
	}

	//////////////////////////////////////////////////////////////////////////////
	///判断平面多边形3个相邻点的中间点(p2)是凹点还是凸点，多边形以逆时针为正向
	///@param p1 顶点1
	///@param p2 顶点2
	///@param p3 顶点3
	///@returns bool: true表示为凸点，false表示为凹点
	//////////////////////////////////////////////////////////////////////////////
	static bool isConvex(const PointType& p1, const PointType& p2, const PointType& p3)
	{
		Vector2<XYType> v1(p2.m_x - p1.m_x, p2.m_y - p1.m_y);
		Vector2<XYType> v2(p3.m_x - p2.m_x, p3.m_y - p2.m_y);

		return (v1 * v2 >= 0);
	}
	static bool isConvex(XYType x1, XYType y1, XYType x2, XYType y2, XYType x3, XYType y3)
	{
		PointType p1(x1, y1);
		PointType p2(x2, y2);
		PointType p3(x3, y3);

		return (isConvex(p1, p2, p3));
	}
	//////////////////////////////////////////////////////////////////////////////
	///计算两个向量的角平分线
	///@param v1 向量1
	///@param v2 向量2
	///@returns Vector2<Real>: 角平分线的单位向量
	static Vector2<Real> PlaneMath::getBisector( const Vector2<Real>& v1, const Vector2<Real>& v2)
	{
		Vector2<Real> a = v1;
		a.normalize();

		Vector2<Real> b = v2;
		b.normalize();

		Vector2<Real> c;

		if (a == b)	//平行且同向
		{
			c = b;
		}
		else if (a == -b)	//平行且反向
		{
			c.m_x = -b.m_y;
            c.m_y = b.m_x;
			}
			else
			{
            c = a + b;
			c.normalize();
		}
		return (c);
	}
	//////////////////////////////////////////////////////////////////////////////
	///计算两射线的交点
	///@param p1 射线1的起点
	///@param p2 射线1的朝向点
	///@param p3 射线2的起点
	///@param p4 射线2的朝向点
	///@param p5 两射线的交点
	///@returns bool: 如果两条射线有交点，返回true，且交点为p5；否则返回false，且p5为无效值
	static bool getRayIntersection(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4, PointType& p5)
	{
		if (!getLineIntersectPoint(p1, p2, p3, p4, p5))
		{
			return (false);
		}

		if ((p2.m_x - p1.m_x) * (p5.m_x - p1.m_x) >= 0 
			&& (p2.m_y - p1.m_y) * (p5.m_y - p1.m_y) >= 0
			&& (p4.m_x - p3.m_x) * (p5.m_x - p3.m_x) >= 0
			&& (p4.m_y - p3.m_y) * (p5.m_y - p3.m_y) >= 0)
		{
			return (true);
		}
		else
		{
			return (false);
		}
	}
	//////////////////////////////////////////////////////////////////////////////
	///计算射线和直线的交点
	///@param p1 射线的起点
	///@param p2 射线的朝向点
	///@param p3 直线起点
	///@param p4 直线终点
	///@param p5 交点
	///@returns bool: 如果有交点，返回true，且交点为p5；否则返回false，且p5为无效值
	static bool getRayLineIntersection(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4, PointType& p5)
	{
		if (!getLineIntersectPoint(p1, p2, p3, p4, p5))
		{
			return (false);
		}

		if ((p2.m_x - p1.m_x) * (p5.m_x - p1.m_x) >= 0 
			&& (p2.m_y - p1.m_y) * (p5.m_y - p1.m_y) >= 0)
		{
			return (true);
		}
		else
		{
			return (false);
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算射线和直线段的交点
	///@param p1 射线的起点
	///@param p2 射线的朝向点
	///@param p3 直线段起点
	///@param p4 直线段终点
	///@param p5 交点
	///@returns bool: 如果有交点，返回true，且交点为p5；否则返回false，且p5为无效值
	static bool getRayLinesegIntersection(const PointType& p1, const PointType& p2,
		const PointType& p3, const PointType& p4, PointType& p5)
	{
		if( getRayLineIntersection(p1,p2,p3,p4,p5 ))
		{
			// 判定p5 是否在线段上
			if( isPointOnSegment( p5, p3, p4 ) )
				return true;
			else
				return false;

		}
		else
		{
			return false;
		}
		
	}

    //////////////////////////////////////////////////////////////////////////////
    ///计算沿着射线方向给定距离d的点坐标
    ///@param p1 射线的起点
    ///@param p2 射线的朝向点
    ///@param d  距离
    ///@param p  射线上距离p1距离为d的点
    ///@returns bool:  执行正常返回true，否则返回false 
    static bool getPointOnRay(const PointType& p1, const PointType& p2, Real d, PointType& p)
    {
        Vector2<XYType> v(p2.m_x - p1.m_x, p2.m_y - p1.m_y);
        if (v.isZero())
        {
            return (false);
        }

        v.normalize();
        v.scaleToLength(d);

        p.m_x = p1.m_x + v.m_x;
        p.m_y = p1.m_y + v.m_y;

        return (true);
    }

	//////////////////////////////////////////////////////////////////////////////
	///去除坐标重复的点，如果多边形的首尾点闭合（重合），则这2个重合点不会被去除
	///@param shp 点串
	///@returns bool: 如果去除了至少1个坐标重复的点，则返回true，否则返回false
	static bool dropSameShape(PointVec& shp)
	{
		if (shp.size() < 3)
		{
			return (false);
		}

		bool rtn = false;

		PointVec out_vec;
		for (size_t i = 0; i < shp.size(); ++i)
		{
			if (out_vec.size() > 0 && shp[i] == out_vec.back())
			{
				rtn = true;
				continue;
			}
			out_vec.push_back(shp[i]);
		}
		shp.swap(out_vec);
		return (rtn);
	}

	//////////////////////////////////////////////////////////////////////////////
	///去除多边形上一条直线上的点，如果多边形上有重复点，可能会计算出错
	///所以在使用该方法前，要保证多边形上没有重复坐标的点
	///@param shp 点串
	///@returns bool: 如果去除了至少1个点，则返回true，否则返回false
	static bool dropPointOnLine(PointVec& shp)
	{
		if (shp.size() < 3)
		{
			return (false);
		}

		bool rtn = false;
		size_t idx_prev = 0;
		size_t idx_cur = 0;
		size_t idx_next = 0;

		PointVec out_vec;
		out_vec.reserve(shp.size());
		for (; idx_cur < shp.size(); ++idx_cur)
		{
			if (idx_cur == 0)
			{
				idx_prev = shp.size() - 1;
				idx_next = idx_cur + 1;
			}
			else if (idx_cur == shp.size() - 1)
			{
				idx_prev = idx_cur - 1;
				idx_next = 0;
			}
			else
			{
				idx_prev = idx_cur - 1;
				idx_next = idx_cur + 1;
			}
			PointType& prev = shp[idx_prev];
			PointType& cur = shp[idx_cur];
			PointType& next = shp[idx_next];
			Vector2<XYType> v1(cur.m_x - prev.m_x, cur.m_y - prev.m_y);
			Vector2<XYType> v2(next.m_x - cur.m_x, next.m_y - cur.m_y);

			if (v1 * v2 == 0)
			{
				if ((cur.m_x - prev.m_x) * (cur.m_x - next.m_x) <= 0
					|| (cur.m_y - prev.m_y) * (cur.m_y - next.m_y) <= 0)
				{
					rtn = true;
				}
				else
				{
					out_vec.push_back(cur);
				}
			}
			else
			{
				out_vec.push_back(cur);
			}
		}

		shp.swap(out_vec);
		return (rtn);
	}
	//////////////////////////////////////////////////////////////////////////////
	///线段和多边形的裁剪，可以获得线段在多边形内的部分或者外面的部分
	///该方法要求多边形首尾点封闭
	///@param p1/p2 线段的端点
	///@param poly_vec 带洞的多边形
	///@param result_vec 裁剪后的结果
	///@param op 截取参数，>=0 保留面外部分，<0 保留面内部分
	///@returns void
	static void clipSegmentByPolygon(const PointType& p1, const PointType p2, const PointVecVec& poly_vec,
		PointVecVec& result_vec, int op)
	{
		if (p1 == p2)
		{
			int pnt_pos = isPointInside(p1, poly_vec);
			if ((pnt_pos < 0 && op < 0) || (pnt_pos > 0 && op >= 0))
			{
				result_vec.push_back(PointVec());
				result_vec.back().push_back(p1);
				result_vec.back().push_back(p2);
			}
			return;
		}

		PointVec all_point_vec;
		PointType result_point;
		all_point_vec.push_back(p1);
		all_point_vec.push_back(p2);
		int is_in_face = -1;
		for (size_t i = 0; i < poly_vec.size(); i++)
		{
			const PointVec& shp_vec = poly_vec[i];
			for (size_t j = 0; j < shp_vec.size() - 1; j++)
			{
				if (getSegmentIntersectPoint(p1.m_x, p1.m_y,
					p2.m_x, p2.m_y,
					shp_vec[j].m_x, shp_vec[j].m_y,
					shp_vec[j + 1].m_x, shp_vec[j + 1].m_y,
					result_point.m_x, result_point.m_y))
				{
					if (result_point == p1 || result_point == p2)
					{
						continue;
					}
					all_point_vec.push_back(result_point);
				}
			}
		}
		std::sort(all_point_vec.begin(), all_point_vec.end());
		PointVec::iterator it_new_end = std::unique(all_point_vec.begin(), all_point_vec.end());
		all_point_vec.erase(it_new_end, all_point_vec.end());

		if (all_point_vec.size() == 2)
		{
			//判断中点是在面内还是面外
			PointType cp;
			cp.m_x = Round<XYType>::get(p1.m_x / 2.0 + p2.m_x / 2.0);
			cp.m_y = Round<XYType>::get(p1.m_y / 2.0 + p2.m_y / 2.0);

			if (isPointInside(cp, poly_vec) > 0)
			{
				if (op >= 0)
				{
					result_vec.push_back(PointVec());
					result_vec.back().push_back(p1);
					result_vec.back().push_back(p2);
				}
			}
			else
			{
				if (op < 0)
				{
					result_vec.push_back(PointVec());
					result_vec.back().push_back(p1);
					result_vec.back().push_back(p2);
				}
			}
		}
		else
		{
			//判断交点序列中第一段是在多边形外还是多边形内
			//如果是外，则依次生成的序列是：外-内-外-内-外。。。
			//如果是内，则依次生成的序列是：内-外-内-外-内。。。
			//最终要保留的根据op参数确定
			PointType cp;
			cp.m_x = Round<XYType>::get(all_point_vec[0].m_x / 2.0 + all_point_vec[1].m_x / 2.0);
			cp.m_y = Round<XYType>::get(all_point_vec[0].m_y / 2.0 + all_point_vec[1].m_y / 2.0);

			size_t i = 0;

			if (isPointInside(cp, poly_vec) < 0)
			{
				if (op >= 0)
				{
					i++;
				}
			}
			else
			{
				if (op < 0)
				{
					i++;
				}
			}
			for (; i < all_point_vec.size() - 1; i += 2)
			{
				result_vec.push_back(PointVec());
				result_vec.back().push_back(all_point_vec[i]);
				result_vec.back().push_back(all_point_vec[i + 1]);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	///计算线段的n等分点坐标
	///@param p1				[in]线段的端点1
	///@param p2				[in]线段的端点2
	///@param iSeg				[in]线段等分的数量
	///@param iNum				[in]计算第n个等分点
	///@returns ShapePoint:	第n个等分点的坐标
	//////////////////////////////////////////////////////////////////////////////
	static PointType calcDivPoint( const PointType& p1, 
		const PointType& p2, UINT32 i_seg, UINT32 i_num )
	{
		assert(i_seg >= 2);
		assert(i_num >= 1);
		assert(i_num < i_seg);

		PointType ptResult;

		ptResult.m_x = Round<XYType>::get((Real(p1.m_x) * i_seg 
			+ Real(i_num) * (p2.m_x-p1.m_x)) / i_seg);
		ptResult.m_y = Round<XYType>::get((Real(p1.m_y) * i_seg
			+ Real(i_num) * (p2.m_y-p1.m_y)) / i_seg);

		return (ptResult);
	}
    //////////////////////////////////////////////////////////////////////////////
    ///计算1条直线的左侧和右侧平行线
    ///@param p1		输入直线上的点1
    ///@param p2		输入直线上的点2
    ///@param d			平行线和原直线的距离
    ///@param left_p1	左侧平行线上的点1，和原点1相对，且连线和输入直线垂直
    ///@param left_p2	左侧平行线上的点2，和原点2相对，且连线和输入直线垂直
    ///@param right_p1	右侧平行线上的点1，和原点1相对，且连线和输入直线垂直
    ///@param right_p2	右侧平行线上的点2，和原点2相对，且连线和输入直线垂直
    ///@returns void:
    //////////////////////////////////////////////////////////////////////////////
    static void getParallelLine(const PointType& p1, const PointType& p2, const XYType& d,
        PointType& left_p1, PointType& left_p2,
        PointType& right_p1, PointType& right_p2)
    {
        Real x1 = static_cast<Real>(p1.m_x);
        Real y1 = static_cast<Real>(p1.m_y);
        Real x2 = static_cast<Real>(p2.m_x);
        Real y2 = static_cast<Real>(p2.m_y);

        if ((d == 0) || (p1 == p2))
        {
            return;
        }
        if (x1 == x2)
        {
            if (y1 < y2)
            {
                //p1->p2从下向上
                //左侧平行线
                left_p1.m_x = p1.m_x - d;
                left_p1.m_y = p1.m_y;

                left_p2.m_x = p2.m_x - d;
                left_p2.m_y = p2.m_y;

                //右侧平行线
                right_p1.m_x = p1.m_x + d;
                right_p1.m_y = p1.m_y;

                right_p2.m_x = p2.m_x + d;
                right_p2.m_y = p2.m_y;
            }
            else
            {
                //p1->p2从上向下
                //左侧平行线
                left_p1.m_x = p1.m_x + d;
                left_p1.m_y = p1.m_y;

                left_p2.m_x = p2.m_x + d;
                left_p2.m_y = p2.m_y;

                //右侧平行线
                right_p1.m_x = p1.m_x - d;
                right_p1.m_y = p1.m_y;

                right_p2.m_x = p2.m_x - d;
                right_p2.m_y = p2.m_y;
            }
        }
        else if (y1 == y2)
        {
            if (x1 < x2)
            {
                //p1->p2从左到右
                left_p1.m_x = p1.m_x;
                left_p1.m_y = p1.m_y + d;

                left_p2.m_x = p2.m_x;
                left_p2.m_y = p2.m_y + d;

                right_p1.m_x = p1.m_x;
                right_p1.m_y = p1.m_y - d;

                right_p2.m_x = p2.m_x;
                right_p2.m_y = p2.m_y - d;
            }
            else
            {
                //p1->p2从右到左
                left_p1.m_x = p1.m_x;
                left_p1.m_y = p1.m_y - d;

                left_p2.m_x = p2.m_x;
                left_p2.m_y = p2.m_y - d;

                right_p1.m_x = p1.m_x;
                right_p1.m_y = p1.m_y + d;

                right_p2.m_x = p2.m_x;
                right_p2.m_y = p2.m_y + d;
            }
        }
        else
        {
            double src_dx = x2 - x1;
            double src_dy = y2 - y1;

            double ak = -src_dx / src_dy;	// -1/k
            double dx = d / sqrt(1 + ak * ak);
            double dy = ak * dx;

            left_p1.m_x = p1.m_x + dx;
            left_p1.m_y = p1.m_y + dy;

            right_p1.m_x = p1.m_x - dx;
            right_p1.m_y = p1.m_y - dy;

            Vector2<Real> v1(left_p1.m_x - p1.m_x, left_p1.m_y - p1.m_y);
            Vector2<Real> v2(p2.m_x - p1.m_x, p2.m_y - p1.m_y);
            if (v1.cross(v2) > 0)   //v1在v2逆时针方向
            {
                //重设左侧
                left_p1.m_x = p1.m_x - dx;
                left_p1.m_y = p1.m_y - dy;

                right_p1.m_x = p1.m_x + dx;
                right_p1.m_y = p1.m_y + dy;
            }

            left_p2.m_x = left_p1.m_x + src_dx;
            left_p2.m_y = left_p1.m_y + src_dy;

            right_p2.m_x = right_p1.m_x + src_dx;
            right_p2.m_y = right_p1.m_y + src_dy;
        }
    }
    //////////////////////////////////////////////////////////////////////////////
    ///计算折线段左右测的平行线，建议使用double类型以提高平行线精度
    ///@param pv1			输入点集合
    ///@param d				距离
    ///@param pv_left		左侧平行线
    ///@param pv_right		右侧平行线
    ///@returns void:
    //////////////////////////////////////////////////////////////////////////////
    static void getParallelLine(const PointVec& pv1, const XYType& d, 
        PointVec& pv_left, PointVec& pv_right)
    {
        pv_left.clear();
        pv_right.clear();
        if (pv1.size() < 2)
        {
            return;
        }

        pv_left.reserve(pv1.size());
        pv_right.reserve(pv1.size());

        PointType first_left_p1;
        PointType first_left_p2;
        PointType first_right_p1;
        PointType first_right_p2;
        PointType second_left_p1;
        PointType second_left_p2;
        PointType second_right_p1;
        PointType second_right_p2;
        size_t i = 0;
        for (size_t j = 1; j <= pv1.size() - 1; ++j)
        {
            //计算相邻两条线段同侧平行线的交点
            if (0 == i)
            {
                getParallelLine(pv1[i], pv1[j], d,
                    first_left_p1, first_left_p2,
                    first_right_p1, first_right_p2);

                pv_left.push_back(first_left_p1);
                pv_right.push_back(first_right_p1);
            }
            else
            {
                first_left_p1 = second_left_p1;
                first_left_p2 = second_left_p2;
                first_right_p1 = second_right_p1;
                first_right_p2 = second_right_p2;
            }
            //如果j还没到达折线的最后一个点，则还可以和后续的线段算交点
            //否则就结束循环
            if (j < pv1.size() - 1)
            {
                getParallelLine(pv1[j], pv1[j + 1], d,
                    second_left_p1, second_left_p2,
                    second_right_p1, second_right_p2);

                PointType inter_p;
                if (getLineIntersectPoint(first_left_p1, first_left_p2,
                    second_left_p1, second_left_p2, inter_p))
                {
                    pv_left.push_back(inter_p);
                }
                else
                {
                    pv_left.push_back(first_left_p2);
                }

                if (getLineIntersectPoint(first_right_p1, first_right_p2,
                    second_right_p1, second_right_p2, inter_p))
                {
                    pv_right.push_back(inter_p);
                }
                else
                {
                    pv_right.push_back(first_right_p2);
                }
            }
            else
            {
                second_left_p2 = first_left_p2;
                second_right_p2 = first_right_p2;
            }

            i = j;
        }

        //最后一个线段的末尾点放入集合
        pv_left.push_back(second_left_p2);
        pv_right.push_back(second_right_p2);
    }
    //////////////////////////////////////////////////////////////////////////////
    ///判断点和线的位置关系，参照方向是由p1->p2
    ///@param p			要判断的点
    ///@param p1		线上的点1
    ///@param p2    	线上的点2
    ///@returns INT8:   p在p1->p2左侧，返回-1，在p1->p2上，返回0，在p1->p2右侧，返回1
    //////////////////////////////////////////////////////////////////////////////
    static INT8 getPointLineRelation(const PointType& p, const PointType& p1, const PointType& p2)
    {
        Vector2<XYType> v1(p2.m_x - p1.m_x, p2.m_y - p1.m_y);
        Vector2<XYType> v2(p.m_x - p1.m_x, p.m_y - p1.m_y);

        Real c = v1 * v2;
        if (c == 0)
        {
            return (0);
        }
        else if (c < 0)
        {
            return (-1);
        }
        else
        {
            return (1);
        }
    }



	// added by lifei
	// 20150720
	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算一个以直线段上某点作为设点，求出垂直射线</summary>
	/// <param>
	/// @<- _p1, 直线的起点
	/// @<- _p2, 直线的终点
	/// @<- _p3, 射线的起点
	/// @-> _p4, 距离射线起点固定距离的某朝向点
	/// @-> _p5, 与_p4 对称的朝向点
	/// </param>
	////////////////////////////////////////////////////////////////////////////////
	static void getVerticalRayLine( const PointType& _p1, const PointType& _p2, const PointType& _p3,
		PointType& _p4, PointType& _p5 )
	{
		// y = kx + b
		//计算直线斜率 k
		if ( fabs( _p2.m_y - _p1.m_y) < DOUBLE_EQUAL_LIMIT  )
		{
			if ( _p1.m_x < _p2.m_x )
			{
				// 直接是垂直线了
				_p4.m_x = _p3.m_x;
				_p4.m_y = _p3.m_y - 10;

				_p5.m_x = _p3.m_x;
				_p5.m_y = _p3.m_y + 10;
			}
			else
			{
				// 直接是垂直线了
				_p4.m_x = _p3.m_x;
				_p4.m_y = _p3.m_y + 10;

				_p5.m_x = _p3.m_x;
				_p5.m_y = _p3.m_y - 10;

			}
			
			return;
		}
		
		XYType k = ( _p2.m_y - _p1.m_y ) / ( _p2.m_x - _p1.m_x );
		
		// 垂直线斜率 k1;
		XYType k1 = -1 / k;

		
		// b = y - kx;
		XYType t = _p3.m_y - k1 * _p3.m_x;

		// 求得射线上以 _p3为原点，计算两个对称的朝向点
		// algorithm : 
		// 以_p3为圆心绘制半径为10的圆，求射线与之的两个交点
		XYType r = 10;
		XYType a = 1 + k1 * k1;
		XYType b = 0;
		XYType c = -r*r;
		
		XYType i_x1 = ( -b + sqrt( b* b - 4* a * c ) ) / ( 2 * a );
		XYType i_y1 = k1 * i_x1 ;

		XYType i_x2 = ( -b - sqrt( b* b - 4* a * c ) ) / ( 2 * a );
		XYType i_y2 = k1 * i_x2 ;

		PointType try_p1 = _p3;  
		PointType try_p2 = _p3;
		try_p1.m_x += i_x1;
		try_p1.m_y += i_y1;

		try_p2.m_x += i_x2;
		try_p2.m_y += i_y2;


		int d1 = getPointLineRelation( try_p1, _p1,_p2 );
		int d2 = getPointLineRelation( try_p2, _p1,_p2 );


		if( d1 < 0 )
		{
			_p4 = try_p1;
			_p5 = try_p2;
		}
		else
		{
			_p4 = try_p2;
			_p5 = try_p1;
		}


	}


	// added by lifei
	// 20150827
	////////////////////////////////////////////////////////////////////////////////
	/// <summary>计算通过三个点所构成圆弧的圆心和半径</summary>
	/// <param>
	/// @<- _p1, 圆弧起点
	/// @<- _p2, 圆弧中点
	/// @<- _p3, 圆弧微点
	/// @-> _p4, 中心
	/// @-> _p5, 半径
	/// return： true： 圆弧构建成功；flase 无法构建圆弧
	/// </param>
	////////////////////////////////////////////////////////////////////////////////
	static bool calcCircle( const PointType& _p1, const PointType& _p2, const PointType& _p3,
		PointType& _p4, double& _r )
	{
		double ab_dist =(double) getTwoPointsDistance( _p1, _p3 );
		double c_to_ab =(double) getPointToSegmentShortcut( _p2, _p1, _p3 );
		// 计算半径g
		double r = ( ( ab_dist / 2) * ( ab_dist / 2 ) + c_to_ab * c_to_ab   ) / ( 2 * c_to_ab );

		//计算圆心
		// 计算C点到ab段的投影点
		XYType c_prj_to_ab_x = 0;
		XYType c_prj_to_ab_y = 0;
		getClosestPointOnSegment( _p1.m_x, _p1.m_y, _p3.m_x, _p3.m_y, _p2.m_x, _p2.m_y, c_prj_to_ab_x, c_prj_to_ab_y );
		PointType c_prj_to_ab ( c_prj_to_ab_x, c_prj_to_ab_y );
		
		PointType circel_pt;

		getPointOnRay( _p2, c_prj_to_ab, r, circel_pt );
		_r = r;
		_p4 = circel_pt;

		return true;

	}

	// added by lifei
	// 20150828
	////////////////////////////////////////////////////////////////////////////////
	/// <summary>判断一个点是否位于一个圆内</summary>
	/// <param>	
	/// @<- _circle_pt, 中心
	/// @<- _r, 半径
	/// @<- _p2,判定点 
	/// return： true：false
	/// </param>
	////////////////////////////////////////////////////////////////////////////////
	static bool inCircle( const PointType& _circle_pt, double _r, const PointType& _pt )
	{
		double ab_dist =(double) getTwoPointsDistance( _circle_pt, _pt );
		return ab_dist < _r;

	}

	//////////////////////////////////////////////////////////////////////////
	///<summary> 计算圆弧从起点顺时针旋转固定角度得到的点坐标
	/// @<- _s_pt 圆弧起点	
	/// @<- _o_pt 圆弧圆心
	/// @<-_r 圆弧半径
	/// @<- _angle 旋转角度
	/// @-> _c_pt 计算所得坐标
	////////////////////////////////////////////////////////////////////////////
	static void calcPointOnArc( const PointType& _s_pt, const PointType& _o_pt, double _r, float _angle, PointType& _c_pt )
	{
		
		// 求得圆弧终点与圆心构成直线与X轴夹角
		XYType dx_s_pt = _s_pt.m_x - _o_pt.m_x;
		XYType dy_s_pt = _s_pt.m_y - _o_pt.m_y;
		// 象限判定
		int area = 1;
		if ( dx_s_pt >=0 && dy_s_pt >=0 )
		{
			area = 1;
		}
		else if ( dx_s_pt < 0 && dy_s_pt >= 0 )
		{
			area = 2;
		}
		else if ( dx_s_pt <0 && dy_s_pt < 0 )
		{
			area = 3;
		}
		else if ( dx_s_pt >=0 && dy_s_pt < 0 )
		{
			area = 4;
		}
		double rate = dy_s_pt / fabs( dx_s_pt ) ;
		double angle = atan(  rate );		
	//	double rate = dy_s_pt / fabs(_r );
	//	double angle = asin(  rate );		


		//依据象限换算角度
		if ( area == 2  || area == 3 )
		{
			angle = PII - angle;
		}
		else if ( area == 4 )
		{
			angle = 2*PII + angle;
		}
		//angle += _angle;
		angle -= _angle * ( _r >0? 1:-1);
		XYType dx_c_pt = cos( angle ) * fabs( _r );
		XYType dy_c_pt = sin( angle )* fabs( _r )   ;
	
		_c_pt.m_x = dx_c_pt + _o_pt.m_x;
		_c_pt.m_y = dy_c_pt + _o_pt.m_y ;	
	}
	////////////////////////////////////////////////////////////////////////////////
	/// <summary>贝塞尔曲线通用算法</summary>
	/// <param>	
	/// @<- _control_pts 确定贝塞尔曲线的控制点集合
	/// @<- _density 密度值（0~1)
	/// @->_bezier 获取的贝塞尔曲线
	/// return： 形点数
	/// </param>
	////////////////////////////////////////////////////////////////////////////////
	static int bezier(const std::vector< PointType >& _control_pts, double _density, std::vector< PointType>& _bezier)
	{
		int n = _control_pts.size() - 1;
		if (n < 2)
		{
			return 0;
		}
		for (double t = 0.0; t < 1.0; t += _density)
		{
			PointType pt;
			pt.m_x = 0;
			pt.m_y = 0;

			for (int i = 0; i <= n; i++)
			{
				int fac_para = fac(n) / (fac(i) * fac(n - i));
				XYType x = _control_pts[i].m_x;
				XYType y = _control_pts[i].m_y;
				pt.m_x += fac_para * x * pow(1 - t, n - i) * pow(t, i);
				pt.m_y += fac_para * y * pow(1 - t, n - i) * pow(t, i);
			}
			_bezier.push_back(pt);
		}
		_bezier.push_back(*_control_pts.rbegin());
		return _bezier.size();
	}
};

}

#endif // _PlaneMath_h_
