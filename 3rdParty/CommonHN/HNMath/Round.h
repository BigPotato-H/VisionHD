#ifndef _ROUND_H_
#define _ROUND_H_
#include <math.h>
#include "MathType.h"
#include "Vector2.h"

namespace HNMath {

	enum ERoundMethod
	{
		ERM_NEAR = 0,
		ERM_TRUNC,
		ERM_FLOOR,
		ERM_CEIL
	};

	//该模板用于double型到整形的转换
	//默认使用四舍五入
	template<typename _Tr, ERoundMethod mth = ERM_NEAR>
	class Round
	{
	public:
		static _Tr get(double d)
		{
			switch (mth)
			{
			case ERM_NEAR:
				return (d > 0.0 ? static_cast<_Tr>(d + 0.5) : static_cast<_Tr>(d - 0.5));
			case ERM_TRUNC:
				return (static_cast<_Tr>(d));
			case ERM_FLOOR:
				return (static_cast<_Tr>(floor(d)));
			case ERM_CEIL:
				return (static_cast<_Tr>(ceil(d)));
			default:
				return (static_cast<_Tr>(d));
			}
		};

		static _Tr get(float d)
		{
			switch (mth)
			{
			case ERM_NEAR:
				return (d > 0.0 ? static_cast<_Tr>(d + 0.5) : static_cast<_Tr>(d - 0.5));
			case ERM_TRUNC:
				return (static_cast<_Tr>(d));
			case ERM_FLOOR:
				return (static_cast<_Tr>(floor(d)));
			case ERM_CEIL:
				return (static_cast<_Tr>(ceil(d)));
			default:
				return (static_cast<_Tr>(d));
			}
		};

		static _Tr get(Real d)
		{
			switch (mth)
			{
			case ERM_NEAR:
				return (d > 0.0 ? static_cast<_Tr>(d + 0.5) : static_cast<_Tr>(d - 0.5));
			case ERM_TRUNC:
				return (static_cast<_Tr>(d));
			case ERM_FLOOR:
				return (static_cast<_Tr>(floor(double(d))));
			case ERM_CEIL:
				return (static_cast<_Tr>(ceil(double(d))));
			default:
				return (static_cast<_Tr>(d));
			}
		}

	private:
		Round() {}
		~Round() {}
	};

	//防止浮点数之间互转出错，做下述特化处理，浮点数之间转换无视mth参数
	template<ERoundMethod mth>
	class Round<double, mth>
	{
	public:
		static double get(double d)
		{
			return (d);
		}
		static double get(float d)
		{
			return (static_cast<double>(d));
		}
		static double get(Real r)
		{
			return (double(r));
		}
	private:
		Round() {}
		~Round() {}
	};
	template<ERoundMethod mth>
	class Round<float, mth>
	{
	public:
		static float get(double d)
		{
			return (static_cast<float>(d));
		}
		static float get(float d)
		{
			return (d);
		}
		static float get(Real r)
		{
			return (float(r));
		}
	private:
		Round() {}
		~Round() {}
	};

	template<ERoundMethod mth>
	class Round<Real, mth>
	{
	public:
		static Real get(double d)
		{
			return (Real(d));
		}
		static Real get(float d)
		{
			return (Real(d));
		}
		static Real get(Real r)
		{
			return (r);
		}
	private:
		Round() {}
		~Round() {}
	};

}

#endif // _Round_h_