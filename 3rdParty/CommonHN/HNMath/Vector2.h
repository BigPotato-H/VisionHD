#ifndef _VECTOR2_H_
#define _VECTOR2_H_
#include "MathType.h"
#include "Coordinate.h"
typedef double DOUBLE;
namespace HNMath {
	//////////////////////////////////////////////////////////////////////////
	///2维矢量定义
	template<typename T>
	class Vector2
	{
	public:
		typedef T	XYType;

	public:
		Vector2() : m_x(T(0)), m_y(T(0))
		{

		}
		Vector2(XYType x, XYType y) : m_x(x), m_y(y)
		{

		}
		Vector2(const Vector2& v) : m_x(v.m_x), m_y(v.m_y)
		{

		}
		Vector2(const TPoint<XYType>& p1, const TPoint<XYType>& p2)
		{
			m_x = p2.m_x - p1.m_x;
			m_y = p2.m_y - p1.m_y;
		}
		~Vector2()
		{
			m_x = T(0);
			m_y = T(0);
		}

	public:
		const Vector2& operator=(const Vector2& v)
		{
			m_x = v.m_x;
			m_y = v.m_y;
			return (*this);
		}
		bool operator==(const Vector2& v) const
		{
			return (m_x == v.m_x && m_y == v.m_y);
		}
		bool operator!=(const Vector2& v) const
		{
			return (m_x != v.m_x || m_y != v.m_y);
		}
		bool operator<(const Vector2& v) const
		{
			if (m_x != v.m_x)
			{
				return (m_x < v.m_x);
			}
			else
			{
				return (m_y < v.m_y);
			}
		}
		bool operator>(const Vector2& v) const
		{
			if (m_x != v.m_x)
			{
				return (m_x > v.m_x);
			}
			else
			{
				return (m_y > v.m_y);
			}
		}
		Vector2 operator+(const Vector2& v) const
		{
			Vector2 tv;
			tv.m_x = m_x + v.m_x;
			tv.m_y = m_y + v.m_y;
			return (tv);
		}
		Vector2 operator-(const Vector2& v) const
		{
			Vector2 tv;
			tv.m_x = m_x - v.m_x;
			tv.m_y = m_y - v.m_y;
			return (tv);
		}
		//矢量叉积
		//若P * Q > 0 则P在Q的顺时针方向
		//若P * Q < 0 则P在Q的逆时针方向
		//若P * Q = 0 则P和Q共线，可能同向也可能反向
		Real operator*(const Vector2& v) const
		{
			return (cross(v));
		}
		//矢量叉积
		Real cross(const Vector2& v) const
		{
			return (Real(m_x) * v.m_y - Real(m_y) * v.m_x);
		}
		//矢量点积
		Real dot(const Vector2& v) const
		{
			return (Real(m_x) * v.m_x + Real(m_y) * v.m_y);
		}
		//与scale相同
		Vector2 operator*(XYType r) const
		{
			Vector2 tv;
			tv.m_x = m_x * r;
			tv.m_y = m_y * r;
			return (tv);
		}
		const Vector2& scale(XYType r)
		{
			m_x *= r;
			m_y *= r;
			return (*this);
		}
		Vector2 operator/(XYType r) const
		{
			Vector2 tv;
			tv.m_x = m_x / r;
			tv.m_y = m_y / r;
			return (tv);
		}
		Vector2 operator/(const Vector2& v) const
		{
			Vector2 tv;
			tv.m_x = m_x / v.m_x;
			tv.m_y = m_y / v.m_y;
			return (tv);
		}
		Vector2 operator-() const
		{
			Vector2 tv;
			tv.m_x = -m_x;
			tv.m_y = -m_y;
			return (tv);
		}
		//求本向量到目标向量的夹角的余弦
		Real cosTo(const Vector2& _v) const
		{
			Vector2 v1(*this);
			Vector2 v2(_v);

			v1.normalize();
			v2.normalize();

			Vector2 v3 = v1 - v2;

			Real a = v1.length();
			Real b = v2.length();
			Real c = v3.length();

			return ((a * a + b * b - c * c) / 2 * a * b);
		}
		//向量长度拉伸，不改变方向
		Real scaleToLength(Real len)
		{
			len = fabs(DOUBLE(len));
			if (m_x == 0)
			{
				m_y = Round<XYType>::get(len);
			}
			else
			{
				Real k = m_y / m_x;
				Real dx = len * sqrt(1.0 / (1.0 + DOUBLE(k * k)));
				dx = m_x > 0 ? dx : -dx;
				Real dy = k * dx;
				m_x = Round<XYType>::get(dx);
				m_y = Round<XYType>::get(dy);
			}
			return (len);
		}

	public:
		Real length()
		{
			return (Real(sqrt(double(m_x) * double(m_x) + double(m_y) * double(m_y))));
		}
		void normalize()
		{
			Real len = length();
			if (len > 0)
			{
				m_x = Round<XYType>::get(Real(m_x) / len);
				m_y = Round<XYType>::get(Real(m_y) / len);
			}
		}
		bool isZero() const
		{
			return (m_x == 0 && m_y == 0);
		}

	public:
		XYType m_x;
		XYType m_y;
	};

	template<>
	bool Vector2<float>::operator ==(const Vector2<float>& v) const
	{
		return (fabs(m_x - v.m_x) < FLT_EPSILON && fabs(m_y - v.m_y) < FLT_EPSILON);
	}
	template<>
	bool Vector2<float>::operator !=(const Vector2<float>& v) const
	{
		return (!(*this == v));
	}
	template<>
	bool Vector2<float>::operator <(const Vector2<float>& v) const
	{
		if (fabs(m_x - v.m_x) > FLT_EPSILON)
		{
			return (m_x < v.m_x);
		}
		else
		{
			return (m_y < v.m_y);
		}
	}
	template<>
	bool Vector2<float>::operator >(const Vector2<float>& v) const
	{
		if (fabs(m_x - v.m_x) > FLT_EPSILON)
		{
			return (m_x > v.m_x);
		}
		else
		{
			return (m_y > v.m_y);
		}
	}

	template<>
	bool Vector2<double>::operator ==(const Vector2<double>& v) const
	{
		return (fabs(m_x - v.m_x) < DBL_EPSILON && fabs(m_y - v.m_y) < DBL_EPSILON);
	}
	template<>
	bool Vector2<double>::operator !=(const Vector2<double>& v) const
	{
		return (!(*this == v));
	}
	template<>
	bool Vector2<double>::operator <(const Vector2<double>& v) const
	{
		if (fabs(m_x - v.m_x) > DBL_EPSILON)
		{
			return (m_x < v.m_x);
		}
		else
		{
			return (m_y < v.m_y);
		}
	}
	template<>
	bool Vector2<double>::operator >(const Vector2<double>& v) const
	{
		if (fabs(m_x - v.m_x) > DBL_EPSILON)
		{
			return (m_x > v.m_x);
		}
		else
		{
			return (m_y > v.m_y);
		}
	}

}
#endif // _Vector2_h_