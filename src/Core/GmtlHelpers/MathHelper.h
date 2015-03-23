#ifndef CORE_GMTL_HELPERS_MATHHELPER_H
#define CORE_GMTL_HELPERS_MATHHELPER_H

#include "..\Core.h"

class MathHelper
{
public:
	static bool			IsZero(float a);
	static bool			IsZero(double a);
	static bool			IsNan(float a);
	static bool			IsNan(double a);
	static bool			IsInf(float a);
	static bool			IsInf(double a);
};

#endif /* CORE_GMTL_HELPERS_MATHHELPER_H */