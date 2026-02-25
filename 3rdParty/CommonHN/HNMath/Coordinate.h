#ifndef COORDINATE_H
#define COORDINATE_H

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include "MathType.h"
#include <vector>
#include <map>
#include <set>
using namespace std;

namespace HNMath{

	// Coordinate
	template<typename T> class TPoint3d;
	typedef TPoint3d<double> PointType;
	typedef vector<PointType > PointVec;


	template<typename T>
	class TPoint
	{	
	public:
		//static UINT32 Rounding_Factor;
		//static T Compare_Precision;
		
	public:
		TPoint() : m_x(T(0)), m_y(T(0)){}
		TPoint(T x, T y) : m_x(x), m_y(y){}
		TPoint( TPoint3d<T> _xyz );

		TPoint(const TPoint& coord) : m_x(coord.m_x), m_y(coord.m_y){}
		TPoint& operator = (const TPoint& coord)
		{
			if (this != &coord)
			{
				m_x = coord.m_x;
				m_y = coord.m_y;
			}
			//
			return(*this);
		}
		~TPoint(){}

	public:
		bool operator==(const TPoint& t) const
		{
			return ((m_x == t.m_x) && (m_y == t.m_y));
		}
		bool operator!=(const TPoint& t) const
		{
			return (!operator==(t));
		}
		bool operator<(const TPoint& t) const
		{
			if (m_x != t.m_x)
			{
				return (m_x < t.m_x);
			}
			else
			{
				return (m_y < t.m_y);
			}
		}
	public:
		T& X() { return m_x; }
		T& Y() { return m_y; }

	public:
		T m_x;
		T m_y;


	};		

	template< typename T> TPoint<T>::TPoint( TPoint3d<T> _xyz )
	{
		m_x = _xyz.m_x;
		m_y = _xyz.m_y;
	}


	//float
	template<>
	bool TPoint<float>::operator==(const TPoint<float>& t) const
	{		
		return Real( m_x ) == Real(t.m_x ) && Real( m_y ) == Real(t.m_y );
	}
	template<>
	bool TPoint<float>::operator!=( const TPoint< float>& t ) const
	{
		return !(operator==( t ) );
	}

	template<>
	bool TPoint<float>::operator <(const TPoint<float>& t) const
	{
		if ( Real(m_x) !=Real( t.m_x )   )
		{
			return Real( m_x ) < Real( t.m_x );
		}
		else if (  Real( m_y ) != Real( t.m_y ) )
		{
			return Real( m_y ) < Real( t.m_y );
		}
		else
		{			
			return false;
		}
		
	}	
	
	// Double	
	template<>
	bool TPoint<double>::operator==(const TPoint<double>& t) const
	{
		return Real( m_x ) == Real(t.m_x ) && Real( m_y ) == Real(t.m_y );
	}
	
	template<>
	bool TPoint<double>::operator!=( const TPoint< double >& t ) const
	{
		return !(operator==( t ) );
	}

	template<>
	bool TPoint<double>::operator <(const TPoint<double>& t) const
	{
		if ( Real(m_x) !=Real( t.m_x )   )
		{
			return Real( m_x ) < Real( t.m_x );
		}
		else if (  Real( m_y ) != Real( t.m_y ) )
		{
			return Real( m_y ) < Real( t.m_y );
		}
		else
		{			
			return false;
		}
	}


	//////////////////////////////////////////////////////////////////////////
	////
	////   具有Z轴的坐标
	////
	template<typename T>
	class TPoint3d
	{	
	public:
		TPoint3d() : m_x(T(0)), m_y(T(0)), m_z(T(0)) {}
		TPoint3d(T x, T y, T z=0 ) : m_x(x), m_y(y), m_z(z) {}
		TPoint3d( const TPoint<T>& _coord )
		{
			m_x = _coord.m_x;
			m_y = _coord.m_y;
			m_z = 0;
		}

		TPoint3d(const TPoint3d& coord) : m_x(coord.m_x), m_y(coord.m_y), m_z( coord.m_z) {}

		TPoint3d& operator = (const TPoint3d& coord)
		{
			if (this != &coord)
			{
				m_x = coord.m_x;
				m_y = coord.m_y;
				m_z = coord.m_z;
			}
			//
			return(*this);
		}
		TPoint3d& operator = (const TPoint<T>& coord)
		{
			m_x = coord.m_x;			
			m_y = coord.m_y;
			m_z = 0;
			
			return(*this);
		}
		~TPoint3d(){}

	public:
		bool operator==(const TPoint3d& t) const
		{
			return ((m_x == t.m_x) && (m_y == t.m_y) && (m_z == t.m_z ) );
		}
		bool operator!=(const TPoint3d& t) const
		{
			return (!operator==(t));
		}
		bool operator<(const TPoint3d& t) const
		{
			if (m_x != t.m_x)
			{
				return (m_x < t.m_x);
			}
			else if ( m_y != t.m_y )
			{
				return (m_y < t.m_y);
			}
			else
			{
				return ( m_z < t.m_z );
			}

		}

		TPoint3d<T> operator-( const TPoint3d& t ) const
		{
			TPoint3d<T> tmp_coord = *this;
			tmp_coord.m_x = tmp_coord.m_x - t.m_x;
			tmp_coord.m_y = tmp_coord.m_y - t.m_y;
			tmp_coord.m_z = tmp_coord.m_z - t.m_z;
			return tmp_coord;			
		}

		TPoint3d<T> operator+( const TPoint3d& t ) const
		{
			TPoint3d<T> tmp_coord = *this;
			tmp_coord.m_x = t.m_x + tmp_coord.m_x;
			tmp_coord.m_y = t.m_y + tmp_coord.m_y;
			tmp_coord.m_z = t.m_z + tmp_coord.m_z;
			return tmp_coord;			
		}

	public:		
		T& X(){ return m_x; }
		T& Y(){ return m_y; }
		T& Z(){ return m_z; }

//<add by xiangrongc 2018/11/08 16:12:17, 使用共用体，为兼容CC平台的坐标存储>
#if 0
	public:
		T m_x;
		T m_y;
		T m_z;
#else
	public:
		union
		{
			struct
			{
				T m_x, m_y, m_z;
			};
			struct
			{
				T x, y, z;
			};
			T u[3];
		};
	public:
		static inline TPoint3d fromArray(const int a[3]) { return TPoint3d(static_cast<T>(a[0]), static_cast<T>(a[1]), static_cast<T>(a[2])); }
		static inline TPoint3d fromArray(const float a[3]) { return TPoint3d(static_cast<T>(a[0]), static_cast<T>(a[1]), static_cast<T>(a[2])); }
		static inline TPoint3d fromArray(const double a[3]) { return TPoint3d(static_cast<T>(a[0]), static_cast<T>(a[1]), static_cast<T>(a[2])); }
#endif
//</add by xiangrongc 2018/11/08 16:12:17, 使用共用体，为兼容CC平台的坐标存储>
	};	

	//////////////////////////////////////////////////////////////////////////

	//float
	template<>
	bool TPoint3d<float>::operator==(const TPoint3d<float>& t) const
	{		
		return Real( m_x ) == Real(t.m_x ) && Real( m_y ) == Real(t.m_y ) && Real( m_z) == Real( t.m_z );
	}
	template<>
	bool TPoint3d<float>::operator!=( const TPoint3d< float>& t ) const
	{
		return !(operator==( t ) );
	}

	template<>
	bool TPoint3d<float>::operator<(const TPoint3d<float>& t) const
	{
		if ( Real(m_x) !=Real( t.m_x )   )
		{
			return Real( m_x ) < Real( t.m_x );
		}
		else if (  Real( m_y ) != Real( t.m_y ) )
		{
			return Real( m_y ) < Real( t.m_y );
		}
		else if ( Real( m_z ) != Real( t.m_z ))
		{
			return Real( m_z ) < Real( t.m_z );
		}
		{			
			return false;
		}

	}	

	// Double	
	template<>
	bool TPoint3d<double>::operator==(const TPoint3d<double>& t) const
	{		
		return Real( m_x ) == Real(t.m_x ) && Real( m_y ) == Real(t.m_y ) && Real( m_z) == Real( t.m_z );
	}
	template<>
	bool TPoint3d<double>::operator!=( const TPoint3d< double>& t ) const
	{
		return !(operator==( t ) );
	}

	template<>
	bool TPoint3d<double>::operator<(const TPoint3d<double>& t) const
	{
		if ( Real(m_x) !=Real( t.m_x )   )
		{
			return Real( m_x ) < Real( t.m_x );
		}
		else if (  Real( m_y ) != Real( t.m_y ) )
		{
			return Real( m_y ) < Real( t.m_y );
		}
		else if ( Real( m_z ) != Real( t.m_z ))
		{
			return Real( m_z ) < Real( t.m_z );
		}
		{			
			return false;
		}

	}	


	// Rectangle
	template<typename T>
	struct KRect
	{
	public:
		typedef TPoint<T>	XYCoord;
		T m_left;
		T m_bottom;
		T m_right;
		T m_top;

	public:
		KRect() : m_left(0), m_bottom(0), m_right(0), m_top(0) {};
		KRect(const KRect& r) : m_left(r.m_left), m_bottom(r.m_bottom), m_right(r.m_right), m_top(r.m_top){}

		KRect(const XYCoord& left_bottom, const XYCoord& right_top) : 
		m_left(left_bottom.m_x), m_bottom(left_bottom.m_y), m_right(right_top.m_x), m_top(right_top.m_y){}

		KRect(const T& left, const T& bottom, const T& right, const T& top) : 
		m_left(left), m_bottom(bottom), m_right(right), m_top(top){}

	public:
		KRect& operator = (const KRect& r)
		{
			if (this != &r)
			{
				m_left = r.m_left;
				m_bottom = r.m_bottom;
				m_right = r.m_right;
				m_top = r.m_top;
			}
			//
			return(*this);
		}

	public:
		bool isEmpty() const
		{
			return XYCoord(m_left, m_bottom ) == XYCoord( m_right, m_top );
		}
		bool isContain(const XYCoord& p) const
		{
			if ((p.m_x >= m_left) && (p.m_x <= m_right)
				&& (p.m_y >= m_bottom) && (p.m_y <= m_top))
			{
				return (true);
			}
			else
			{
				return (false);
			}
		}
		bool isCross(const KRect<T>& r) const
		{
			bool r1 = isContain( TPoint<T>( r.m_left, r.m_bottom) );
			bool r2 = isContain( TPoint<T>( r.m_right, r.m_bottom) );
			bool r3 = isContain( TPoint<T>( r.m_left, r.m_top) );
			bool r4 = isContain( TPoint<T>( r.m_right, r.m_top) );
			if ( r1 | r2 | r3 | r4  )
			{
				return true;
			}
			else
			{
				//相交计算
				// 构建目标矩形
				TPoint<T> lb_pt ( r.m_left, r.m_bottom );
				TPoint<T> rb_pt( r.m_right, r.m_bottom );
				TPoint<T> rt_pt( r.m_right, r.m_top );
				TPoint<T> lt_pt(  r.m_left, r.m_top );
				vector< vector< TPoint<T> > > complex_polygon(1);

				vector< TPoint< T > >& polygon = *complex_polygon.rbegin();;
				polygon.push_back( lb_pt );
				polygon.push_back( rb_pt );
				polygon.push_back( rt_pt );
				polygon.push_back( lt_pt );
				polygon.push_back( lb_pt );

				

				TPoint<T> lb_pt1 ( m_left, m_bottom );
				TPoint<T> rb_pt1( m_right, m_bottom );
				TPoint<T> rt_pt1( m_right, m_top );
				TPoint<T> lt_pt1(  m_left, m_top );
				vector< TPoint<T> > cross_pts;
				
				
				PlaneMath< double, TPoint>::getSegmentAndFaceIntersections( complex_polygon, lb_pt1, rb_pt1, cross_pts );
				PlaneMath< double, TPoint>::getSegmentAndFaceIntersections( complex_polygon, rb_pt1, rt_pt1, cross_pts );
				PlaneMath< double, TPoint>::getSegmentAndFaceIntersections( complex_polygon, rt_pt1, lt_pt1, cross_pts );
				PlaneMath< double, TPoint>::getSegmentAndFaceIntersections( complex_polygon, lt_pt1, lb_pt1, cross_pts );
				if ( cross_pts.size() > 1 )
				{
					return true;
				}
				else
				{
					return false;
				}


			}
				
		}
	};

	template<>
	bool KRect<double>::isContain( const TPoint<double>& p ) const
	{		
		if ( ( Real( p.m_x) >= Real( m_left) ) && ( Real( p.m_x) <= Real( m_right ) ) &&
			 ( Real( p.m_y) >= Real( m_bottom ) ) && ( Real( p.m_y) <= Real( m_top )) )
		{
			return (true);
		}
		else
		{
			return (false);
		}		
	}
	template<typename T>
	void getPolyLineBox(  std::vector< TPoint<T> > _polyline, KRect<T>& _box )
	{
		set< T > x_set;
		set< T > y_set;
		vector< TPoint<T> >::const_iterator citr = _polyline.begin();
		for (; citr != _polyline.end()	; ++citr )
		{
			const TPoint<T>& pt = *citr;
			x_set.insert( pt.m_x);
			y_set.insert( pt.m_y);
		}
		_box.m_left = * (x_set.begin());
		_box.m_right = *( x_set.rbegin() );
		_box.m_bottom = *( y_set.begin() );
		_box.m_top = *( y_set.rbegin() );

		return ;	

	}

	template<typename T>
	void getPolyLineBox(  std::vector< TPoint3d<T> > _polyline, KRect<T>& _box, T _expand_threshold = 0 )
	{
		set< T > x_set;
		set< T > y_set;
		vector< TPoint3d<T> >::const_iterator citr = _polyline.begin();
		for (; citr != _polyline.end()	; ++citr )
		{
			const TPoint3d<T>& pt = *citr;
			x_set.insert( pt.m_x);
			y_set.insert( pt.m_y);
		}

		_box.m_left = * (x_set.begin());
		_box.m_right = *( x_set.rbegin() );
		_box.m_bottom = *( y_set.begin() );
		_box.m_top = *( y_set.rbegin() );

		_box.m_left -= _expand_threshold;
		_box.m_bottom -= _expand_threshold;
		_box.m_right += _expand_threshold;
		_box.m_top += _expand_threshold;

		return ;	

	}
	template<typename T >
	KRect<T> UnionRect( const KRect< T >& _rect1 , const KRect<T>& _rect2 )
	{
		set< T > x_set;
		set< T > y_set;
		x_set.insert( _rect1.m_left );
		x_set.insert( _rect1.m_right ); 
		x_set.insert( _rect2.m_left );
		x_set.insert(	_rect2.m_right );
		
		y_set.insert( _rect1.m_top);
		y_set.insert( _rect1.m_bottom);
		y_set.insert( _rect2.m_top);
		y_set.insert( _rect2.m_bottom );

		T left = * (x_set.begin());
		T right = *( x_set.rbegin() );
		T bottom = *( y_set.begin() );
		T top = *( y_set.rbegin() );

		return KRect<T>( left, bottom, right, top );

	}

}
#endif
