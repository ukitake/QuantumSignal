#include "Angle.h"
#include "..\GmtlHelpers\QuatHelper.h"

Angle angle_zero = Angle::FromDegrees(0.0);

Angle Angle::FromDegrees(double degrees)
{
	return Angle(degrees, gmtl::Math::deg2Rad(degrees));
}

Angle Angle::FromRadians(double radians)
{
	return Angle(gmtl::Math::rad2Deg(radians), radians);
}

Angle Angle::MidAngle(const Angle& a1, const Angle& a2)
{
	return Angle::FromDegrees(0.5 * (a1.degrees + a2.degrees));
}

double Angle::NormalizedDegreesLatitude(double degrees)
{
	double lat = fmod(degrees, 180.0);
	return lat > 90.0 ? 180.0 - lat : lat < -90.0 ? -180.0 - lat : lat;
}

double Angle::NormalizedDegreesLongitude(double degrees)
{
	double lon = fmod(degrees, 360.0);
	return lon > 180.0 ? lon - 360.0 : lon < -180.0 ? 360.0 + lon : lon;
}

Angle Angle::NormalizedLatitude(const Angle& unnormalizedAngle)
{
	return Angle::FromDegrees(NormalizedDegreesLatitude(unnormalizedAngle.degrees));
}

Angle Angle::NormalizedLongitude(const Angle& unnormalizedAngle)
{
	return Angle::FromDegrees(NormalizedDegreesLongitude(unnormalizedAngle.degrees));
}

Angle Angle::Lerp(const Angle& a1, const Angle& a2, double factor)
{
	if (factor < 0.0) return a1;
	if (factor > 1.0) return a2;

	gmtl::Quatd quat;
	gmtl::slerp(
		quat,
		factor,
		QuatHelper::FromAxisAngle(a1, 1.0, 0.0, 0.0),
		QuatHelper::FromAxisAngle(a2, 1.0, 0.0, 0.0));
	
	Angle angle = QuatHelper::GetRotationX(quat);

	//if (Double.isNaN(angle.degrees))
	//	return null;

	return angle;
}

Angle Angle::AngularDistanceTo(const Angle& angle) const
{
	double differenceDegrees = (angle - *this).degrees;

	if (differenceDegrees < -180)
		differenceDegrees += 360;
	else if (differenceDegrees > 180)
		differenceDegrees -= 360;

	double absAngle = gmtl::Math::abs(differenceDegrees);
	return Angle::FromDegrees(absAngle);
}

Angle Angle::NormalizedLatitude() const
{
	return NormalizedLatitude(*this);
}

Angle Angle::NormalizedLongitude() const
{
	return NormalizedLongitude(*this);
}

inline Angle Angle::operator-() const
{
	return Angle::FromDegrees(-degrees);
}

inline Angle Angle::operator+(const Angle& angle) const
{
	return Angle::FromDegrees(degrees + angle.degrees);
}

inline Angle Angle::operator-(const Angle& angle) const
{
	return Angle::FromDegrees(degrees - angle.degrees);
}

inline Angle Angle::operator*(const Angle& angle) const
{
	return Angle::FromDegrees(degrees * angle.degrees);
}

inline Angle Angle::operator*(double multFactor) const
{
	return Angle::FromDegrees(degrees * multFactor);
}

inline Angle Angle::operator/(const Angle& angle) const
{
	return Angle::FromDegrees(degrees / angle.degrees);
}

inline Angle Angle::operator/(double divFactor) const
{
	return Angle::FromDegrees(degrees / divFactor);
}

inline bool Angle::operator==(const Angle& angle) const
{
	return degrees == angle.degrees;
}

inline bool Angle::operator!=(const Angle& angle) const
{
	return degrees != angle.degrees;
}