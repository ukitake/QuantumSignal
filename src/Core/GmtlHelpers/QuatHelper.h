#ifndef CORE_GMTL_HELPERS_QUATHELPER_H
#define CORE_GMTL_HELPERS_QUATHELPER_H

#include "..\Core.h"

class Angle;
class LatLon;

class QuatHelper
{
public:
	static const gmtl::Quatd	IDENTITY;
	
	static gmtl::Quatd			FromAxisAngle(const Angle& angle, gmtl::Vec4d axis);
	static gmtl::Quatd			FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ);

	static gmtl::Quatd			FromMatrix(const gmtl::Matrix44d& matrix);
	static gmtl::Quatd			FromRotationXYZ(const Angle& x, const Angle& y, const Angle& z);
	static gmtl::Quatd			FromLatLon(const Angle& latitude, const Angle& longitude);

	static Angle				GetAngle(const gmtl::Quatd& quat);
	static gmtl::Vec4d			GetAxis(const gmtl::Quatd& quat);
	static Angle				GetRotationX(const gmtl::Quatd& quat);
	static Angle				GetRotationY(const gmtl::Quatd& quat);
	static Angle				GetRotationZ(const gmtl::Quatd& quat);
	static LatLon				GetLocation(const gmtl::Quatd& quat);

private:
	QuatHelper() { }

	static gmtl::Quatd			FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ, bool normalize);
};

#endif /* CORE_GMTL_HELPERS_QUATHELPER_H */