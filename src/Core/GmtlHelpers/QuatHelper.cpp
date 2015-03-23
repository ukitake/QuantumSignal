#include "QuatHelper.h"
#include "MathHelper.h"
#include "..\Geometry\Angle.h"
#include "..\Geometry\LatLon.h"

gmtl::Quatd QuatHelper::FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ, bool normalize)
{
	if (normalize)
	{
		double length = gmtl::Math::sqrt((axisX * axisX) + (axisY * axisY) + (axisZ * axisZ));
		if (!MathHelper::IsZero(length) && (length != 1.0))
		{
			axisX /= length;
			axisY /= length;
			axisZ /= length;
		}
	}

	double s = angle.SinHalfAngle();
	double c = angle.CosHalfAngle();
	return gmtl::Quatd(axisX * s, axisY * s, axisZ * s, c);
}

gmtl::Quatd QuatHelper::FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ)
{
	return FromAxisAngle(angle, axisX, axisY, axisZ, true);
}

gmtl::Quatd QuatHelper::FromAxisAngle(const Angle& angle, gmtl::Vec4d axis)
{
	return FromAxisAngle(angle, axis[0], axis[1], axis[2]);
}

gmtl::Quatd QuatHelper::FromMatrix(const gmtl::Matrix44d& matrix)
{
	double t = 1.0 + matrix[0][0] + matrix[1][1] + matrix[2][2];
	double x, y, z, w;
	double s;
	if (t > gmtl::GMTL_EPSILON)
	{
		s = 2.0 * gmtl::Math::sqrt(t);
		x = (matrix[1][2] - matrix[2][1]) / s;
		y = (matrix[2][0] - matrix[0][2]) / s;
		z = (matrix[0][1] - matrix[1][0]) / s;
		w = s / 4.0;
	}
	else if ((matrix[0][0] > matrix[1][1]) && (matrix[0][0] > matrix[2][2]))
	{
		s = 2.0 * gmtl::Math::sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]);
		x = s / 4.0;
		y = (matrix[0][1] + matrix[1][0]) / s;
		z = (matrix[2][0] + matrix[0][2]) / s;
		w = (matrix[1][2] - matrix[2][1]) / s;
	}
	else if (matrix[1][1] > matrix[2][2])
	{
		s = 2.0 * gmtl::Math::sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]);
		x = (matrix[0][1] + matrix[1][0]) / s;
		y = s / 4.0;
		z = (matrix[1][2] + matrix[2][1]) / s;
		w = (matrix[2][0] - matrix[0][2]) / s;
	}
	else
	{
		s = 2.0 * gmtl::Math::sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]);
		x = (matrix[2][0] + matrix[0][2]) / s;
		y = (matrix[1][2] + matrix[2][1]) / s;
		z = s / 4.0;
		w = (matrix[0][1] - matrix[1][0]) / s;
	}

	return gmtl::Quatd(x, y, z, w);
}

gmtl::Quatd QuatHelper::FromRotationXYZ(const Angle& x, const Angle& y, const Angle& z)
{
	double cx = x.CosHalfAngle();
	double cy = y.CosHalfAngle();
	double cz = z.CosHalfAngle();
	double sx = x.SinHalfAngle();
	double sy = y.SinHalfAngle();
	double sz = z.SinHalfAngle();

	// The order in which the three Euler angles are applied is critical. This can be thought of as multiplying
	// three quaternions together, one for each Euler angle (and corresponding unit axis). Like matrices,
	// quaternions affect vectors in reverse order. For example, suppose we construct a quaternion
	//     Q = (QX * QX) * QZ
	// then transform some vector V by Q. This can be thought of as first transforming V by QZ, then QY, and
	// finally by QX. This means that the order of quaternion multiplication is the reverse of the order in which
	// the Euler angles are applied.
	//
	// The ordering below refers to the order in which angles are applied.
	//
	// QX = (sx, 0,  0,  cx)
	// QY = (0,  sy, 0,  cy)
	// QZ = (0,  0,  sz, cz)
	//
	// 1. XYZ Ordering
	// (QZ * QY * QX)
	// qw = (cx * cy * cz) + (sx * sy * sz);
	// qx = (sx * cy * cz) - (cx * sy * sz);
	// qy = (cx * sy * cz) + (sx * cy * sz);
	// qz = (cx * cy * sz) - (sx * sy * cz);
	//
	// 2. ZYX Ordering
	// (QX * QY * QZ)
	// qw = (cx * cy * cz) - (sx * sy * sz);
	// qx = (sx * cy * cz) + (cx * sy * sz);
	// qy = (cx * sy * cz) - (sx * cy * sz);
	// qz = (cx * cy * sz) + (sx * sy * cz);
	//

	double qw = (cx * cy * cz) + (sx * sy * sz);
	double qx = (sx * cy * cz) - (cx * sy * sz);
	double qy = (cx * sy * cz) + (sx * cy * sz);
	double qz = (cx * cy * sz) - (sx * sy * cz);

	return gmtl::Quatd(qx, qy, qz, qw);
}

gmtl::Quatd QuatHelper::FromLatLon(const Angle& latitude, const Angle& longitude)
{
	double clat = latitude.CosHalfAngle();
	double clon = longitude.CosHalfAngle();
	double slat = latitude.SinHalfAngle();
	double slon = longitude.SinHalfAngle();

	// The order in which the lat/lon angles are applied is critical. This can be thought of as multiplying two
	// quaternions together, one for each lat/lon angle. Like matrices, quaternions affect vectors in reverse
	// order. For example, suppose we construct a quaternion
	//     Q = QLat * QLon
	// then transform some vector V by Q. This can be thought of as first transforming V by QLat, then QLon. This
	// means that the order of quaternion multiplication is the reverse of the order in which the lat/lon angles
	// are applied.
	//
	// The ordering below refers to order in which angles are applied.
	//
	// QLat = (0,    slat, 0, clat)
	// QLon = (slon, 0,    0, clon)
	//
	// 1. LatLon Ordering
	// (QLon * QLat)
	// qw = clat * clon;
	// qx = clat * slon;
	// qy = slat * clon;
	// qz = slat * slon;
	//
	// 2. LonLat Ordering
	// (QLat * QLon)
	// qw = clat * clon;
	// qx = clat * slon;
	// qy = slat * clon;
	// qz = - slat * slon;
	//

	double qw = clat * clon;
	double qx = clat * slon;
	double qy = slat * clon;
	double qz = 0.0 - slat * slon;

	return gmtl::Quatd(qx, qy, qz, qw);
}

Angle QuatHelper::GetAngle(const gmtl::Quatd& quat)
{
	double w = quat[3];

	double length = gmtl::length(quat);
	if (!MathHelper::IsZero(length) && (length != 1.0))
		w /= length;

	double radians = 2.0 * gmtl::Math::aCos(w);
	
	//if (Double.isNaN(radians))
	//	return null;

	return Angle::FromRadians(radians);
}

gmtl::Vec4d QuatHelper::GetAxis(const gmtl::Quatd& quat)
{
	double x = quat[0];
	double y = quat[1];
	double z = quat[2];

	double length = gmtl::length(quat);
	if (!MathHelper::IsZero(length) && (length != 1.0))
	{
		x /= length;
		y /= length;
		z /= length;
	}

	double vecLength = gmtl::Math::sqrt((x * x) + (y * y) + (z * z));
	if (!MathHelper::IsZero(vecLength) && (vecLength != 1.0))
	{
		x /= vecLength;
		y /= vecLength;
		z /= vecLength;
	}

	return gmtl::Vec4d(x, y, z, 0.0);
}

Angle QuatHelper::GetRotationX(const gmtl::Quatd& quat)
{
	double radians = gmtl::Math::aTan2((2.0 * quat[0] * quat[3]) - (2.0 * quat[1] * quat[2]),
		1.0 - 2.0 * (quat[0] * quat[0]) - 2.0 * (quat[2] * quat[2]));
	
	//if (Double.isNaN(radians))
	//	return null;

	return Angle::FromRadians(radians);
}

Angle QuatHelper::GetRotationY(const gmtl::Quatd& quat)
{
	double radians = gmtl::Math::aTan2((2.0 * quat[1] * quat[3]) - (2.0 * quat[0] * quat[2]),
		1.0 - (2.0 * quat[1] * quat[1]) - (2.0 * quat[2] * quat[2]));
	
	//if (Double.isNaN(radians))
	//	return null;

	return Angle::FromRadians(radians);
}

Angle QuatHelper::GetRotationZ(const gmtl::Quatd& quat)
{
	double radians = gmtl::Math::aSin((2.0 * quat[0] * quat[1]) + (2.0 * quat[2] * quat[3]));
	
	//if (Double.isNaN(radians))
	//	return null;

	return Angle::FromRadians(radians);
}

LatLon QuatHelper::GetLocation(const gmtl::Quatd& quat)
{
	double latRadians = gmtl::Math::aSin((2.0 * quat[1] * quat[3]) - (2.0 * quat[0] * quat[2]));
	double lonRadians = gmtl::Math::aTan2((2.0 * quat[1] * quat[2]) + (2.0 * quat[0] * quat[3]),
		(quat[3] * quat[3]) - (quat[0] * quat[0]) - (quat[1] * quat[1]) + (quat[2] * quat[2]));
	
	//if (Double.isNaN(latRadians) || Double.isNaN(lonRadians))
	//	return null;

	return LatLon::FromRadians(latRadians, lonRadians);
}