#ifndef _MATHTYPE_H
#define _MATHTYPE_H
#include <math.h>
#include <float.h>
#include <basetsd.h>

namespace HNMath {
#define DOUBLE_EQUAL_LIMIT	1.0e-010

#define  DBL_INVALIDATE 0.0 / pow( 0.0, 1.0 );
#define  FLT_INVALIDATE 0.0f /pow( 0.0f, 1.0f );

	// == Added by lifei for chk double is validate 20150908===Begin->
	inline bool validateDBL(double v)
	{
		if (_isnan(v) || v + 1 == v - 1)
		{
			return false;
		}
		else
		{
			return true;
		}

	}
	// == Added by lifei for chk double is validate 20150908===<-End


	class Real
	{
	private:
		double m_value;

	private:
		//计算2浮点数的相对差值，基数以较大的浮点数为准
		//所以其结果总是<1.0的
		//相对差值足够小的话，可以认为2个浮点数相等
		double getRelativeTolerance(double d, double r) const
		{
			double rd = fabs(d) > fabs(r) ? fabs(d) : fabs(r);
			if (rd > DOUBLE_EQUAL_LIMIT)
			{
				//防止r或d为0，计算相对差
				return (fabs((d - r) / rd));
			}
			else
			{
				return (0.0);
			}
		}
	public:

		Real() : m_value(0.0) {}

		template<typename T>
		Real(T val) : m_value(double(val)) {}

		~Real() {}

		template<typename T>
		Real& operator = (T val) { m_value = double(val); return *this; }

		template<typename T>
		operator T() { return T(m_value); }

		Real operator + () const { return *this; }

		template<typename T>
		Real operator + (T f) const { return Real(m_value + f); }
		template<>
		Real operator + (Real f) const { return Real(m_value + f.m_value); }

		template<typename T>
		Real& operator += (T f) { m_value += f; return *this; }
		template<>
		Real& operator += (Real f) { m_value += f.m_value; return *this; }

		Real operator - () const { return Real(-m_value); }

		template<typename T>
		Real operator - (T f) const { return Real(m_value - f); }
		template<>
		Real operator - (Real f) const { return Real(m_value - f.m_value); }

		template<typename T>
		Real& operator -= (T f) { m_value -= f; return *this; }
		template<>
		Real& operator -= (Real r) { m_value -= r.m_value; return *this; }

		template<typename T>
		Real operator * (T f) const { return Real(m_value * f); }
		template<>
		Real operator * (Real f) const { return Real(m_value * f.m_value); }

		template<typename T>
		Real& operator *= (T f) { m_value *= f; return *this; }
		template<>
		Real& operator *= (Real f) { m_value *= f.m_value; return *this; }

		template<typename T>
		Real operator / (T f) const { return Real(m_value / f); }
		template<>
		Real operator / (Real f) const { return Real(m_value / f.m_value); }

		template<typename T>
		Real& operator /= (T f) { m_value /= f; return *this; }
		template<>
		Real& operator /= (Real f) { m_value /= f.m_value; return *this; }

		template<typename T>
		bool operator <  (T r) const { return m_value < r && getRelativeTolerance(m_value, r) > DOUBLE_EQUAL_LIMIT; }
		template<>
		bool operator <  (Real r) const { return operator<(r.m_value); }

		template<typename T>
		bool operator <= (T r) const { return (*this < r || *this == r); }
		template<>
		bool operator <= (Real r) const { return (*this < r || *this == r); }

		//比较相等只考虑相对误差
		template<typename T>
		bool operator == (T r) const
		{
#if 1
			return (getRelativeTolerance(m_value, r) < DOUBLE_EQUAL_LIMIT);
#else
			return (fabs(m_value - r) < DBL_EPSILON);
#endif
		}
		template<>
		bool operator == (Real r) const { return operator==(r.m_value); }

		template<typename T>
		bool operator != (T r) const { return !(m_value == r); }
		template<>
		bool operator != (Real r) const { return !(m_value == r.m_value); }

		template<typename T>
		bool operator >= (T r) const { return (*this > r || *this == r); }
		template<>
		bool operator >= (Real r) const { return (*this > r || *this == r); }

		template<typename T>
		bool operator >  (T r) const { return m_value > r && getRelativeTolerance(m_value, r) > DOUBLE_EQUAL_LIMIT; }
		template<>
		bool operator >  (Real r) const { return operator > (r.m_value); }
	};

	//数学常量
	static const double PII = 3.1415926535897932384626433832795;
	static const Real PI = PII;
	static const Real DE2RA = 0.01745329252;					//角度到弧度的转换系数
	static const Real FLATTENING = 1.000000 / 298.257223563;	// Earth flattening (WGS84)
	static const Real ERAD = 6378.137;						    // 地球半径

	//浮点数常量
	static const UINT32 _UINT32_FOR_NAN = 0x7FF80000;           //用于定义浮点无效值的整形，不直接使用
	static const float FLT_NAN = *(float*)&_UINT32_FOR_NAN;     //NAN(Not a Number) for float

	static const UINT32 _UINT32_FOR_INF = 0x7FF00000;
	static const float FLT_INF = *(float*)&_UINT32_FOR_INF;     //INF(infinite) for float

	static const UINT64 _UINT64_FOR_NAN = 0x7FF8000000000000;
	static const double DBL_NAN = *(double*)&_UINT64_FOR_NAN;   //NAN(Not a Number) for double

	static const UINT64 _UINT64_FOR_INF = 0x7FF0000000000000;
	static const double DBL_INF = *(double*)&_UINT64_FOR_INF;   //INF(infinite) for double

	inline int fac(int n)
	{
		if (n == 0 || n == 1) return 1;
		else return n*fac(n - 1);
	}

}
#endif // _MathType_h_