#include "MathHelper.h"

bool MathHelper::IsZero(float a)
{
	return gmtl::Math::isEqual(a, 0.0f, gmtl::GMTL_EPSILON);
}

bool MathHelper::IsZero(double a)
{
	return gmtl::Math::isEqual(a, 0.0, 1e-15);
}

bool MathHelper::IsNan(float a)
{
	return _isnanf(a) != 0;
}

bool MathHelper::IsNan(double a)
{
	return _isnan(a) != 0;
}

bool MathHelper::IsInf(float a)
{
	return isinf(a);
}

bool MathHelper::IsInf(double a)
{
	return isinf(a);
}