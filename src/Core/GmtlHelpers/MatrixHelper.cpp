#include "MatrixHelper.h"
#include "MathHelper.h"

using namespace gmtl;

gmtl::Matrix44d	MatrixHelper::FromAxes(gmtl::Vec3d xAxis, gmtl::Vec3d yAxis, gmtl::Vec3d zAxis)
{
	normalize(xAxis);
	Vec3d s = xAxis;

	Vec3d f;
	cross(f, s, yAxis);
	normalize(f);

	Vec3d u;
	cross(u, f, s);
	normalize(u);

	double dat[16] = 
	{
		/*s.x, u.x, f.x, 0.0,
		  s.y, u.y, f.y, 0.0,
		  s.z, u.z, f.z, 0.0,
		  0.0, 0.0, 0.0, 1.0,*/

		// gmtl stores matrices column major so I purposefully transposed the data

		s[0], s[1], s[2], 0.0,
		u[0], u[1], u[2], 0.0,
		f[0], f[1], f[2], 0.0,
		0.0, 0.0, 0.0, 1.0,
	};

	Matrix44d ret;
	ret.set(dat);
	return ret;
}

Matrix44d MatrixHelper::FromAxisAngle(const Angle& angle, const gmtl::Vec3d& axis)
{
	return FromAxisAngle(angle, axis[0], axis[1], axis[2], true);
}

Matrix44d MatrixHelper::FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ)
{
	return FromAxisAngle(angle, axisX, axisY, axisZ, true);
}

Matrix44d MatrixHelper::FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ, bool normalize)
{
	if (normalize)
	{
		double length = Math::sqrt((axisX * axisX) + (axisY * axisY) + (axisZ * axisZ));
		if (!MathHelper::IsZero(length) && (length != 1.0))
		{
			axisX /= length;
			axisY /= length;
			axisZ /= length;
		}
	}

	double c = angle.Cos();
	double s = angle.Sin();
	double one_minus_c = 1.0 - c;

	double data[16] = 
	{
		//// Row 1
		//c + (one_minus_c * axisX * axisX),
		//(one_minus_c * axisX * axisY) - (s * axisZ),
		//(one_minus_c * axisX * axisZ) + (s * axisY),
		//0.0,
		//// Row 2
		//(one_minus_c * axisX * axisY) + (s * axisZ),
		//c + (one_minus_c * axisY * axisY),
		//(one_minus_c * axisY * axisZ) - (s * axisX),
		//0.0,
		//// Row 3
		//(one_minus_c * axisX * axisZ) - (s * axisY),
		//(one_minus_c * axisY * axisZ) + (s * axisX),
		//c + (one_minus_c * axisZ * axisZ),
		//0.0,
		//// Row 4
		//0.0, 0.0, 0.0, 1.0,

		// Column 1
		c + (one_minus_c * axisX * axisX),
		(one_minus_c * axisX * axisY) + (s * axisZ),
		(one_minus_c * axisX * axisZ) - (s * axisY),
		0.0,

		// Column 2
		(one_minus_c * axisX * axisY) - (s * axisZ),
		c + (one_minus_c * axisY * axisY),
		(one_minus_c * axisY * axisZ) + (s * axisX),
		0.0,

		// Column 3
		(one_minus_c * axisX * axisZ) + (s * axisY),
		(one_minus_c * axisY * axisZ) - (s * axisX),
		c + (one_minus_c * axisZ * axisZ),
		0.0,

		// Column 4
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	return ret;
}

Matrix44d MatrixHelper::FromQuaternion(const gmtl::Quatd& quaternion)
{
	return FromQuaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3], true);
}

Matrix44d MatrixHelper::FromQuaternion(double x, double y, double z, double w, bool normalize)
{
	if (normalize)
	{
		double length = Math::sqrt((x * x) + (y * y) + (z * z) + (w * w));
		if (!MathHelper::IsZero(length) && (length != 1.0))
		{
			x /= length;
			y /= length;
			z /= length;
			w /= length;
		}
	}

	double data[16] = 
	{
		//// Row 1
		//1.0 - (2.0 * y * y) - (2.0 * z * z),
		//(2.0 * x * y) - (2.0 * z * w),
		//(2.0 * x * z) + (2.0 * y * w),
		//0.0,
		//// Row 2
		//(2.0 * x * y) + (2.0 * z * w),
		//1.0 - (2.0 * x * x) - (2.0 * z * z),
		//(2.0 * y * z) - (2.0 * x * w),
		//0.0,
		//// Row 3
		//(2.0 * x * z) - (2.0 * y * w),
		//(2.0 * y * z) + (2.0 * x * w),
		//1.0 - (2.0 * x * x) - (2.0 * y * y),
		//0.0,
		//// Row 4
		//0.0, 0.0, 0.0, 1.0,

		1.0 - (2.0 * y * y) - (2.0 * z * z),
		(2.0 * x * y) + (2.0 * z * w),
		(2.0 * x * z) - (2.0 * y * w),
		0.0,

		(2.0 * x * y) - (2.0 * z * w),
		1.0 - (2.0 * x * x) - (2.0 * z * z),
		(2.0 * y * z) + (2.0 * x * w),
		0.0,

		(2.0 * x * z) + (2.0 * y * w),
		(2.0 * y * z) - (2.0 * x * w),
		1.0 - (2.0 * x * x) - (2.0 * y * y),
		0.0,

		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	return ret;
}

Matrix44d MatrixHelper::FromRotationXYZ(const Angle& xRotation, const Angle& yRotation, const Angle& zRotation)
{
	double cx = xRotation.Cos();
	double cy = yRotation.Cos();
	double cz = zRotation.Cos();
	double sx = xRotation.Sin();
	double sy = yRotation.Sin();
	double sz = zRotation.Sin();

	double data[16] = 
	{
		/*cy * cz,
		-cy * sz,
		sy,
		0.0,

		(sx * sy * cz) + (cx * sz),
		-(sx * sy * sz) + (cx * cz),
		-sx * cy,
		0.0,

		-(cx * sy * cz) + (sx * sz),
		(cx * sy * sz) + (sx * cz),
		cx * cy,
		0.0,

		0.0, 0.0, 0.0, 1.0,*/

		cy * cz,
		(sx * sy * cz) + (cx * sz),
		-(cx * sy * cz) + (sx * sz),
		0.0,

		-cy * sz,
		-(sx * sy * sz) + (cx * cz),
		(cx * sy * sz) + (sx * cz),
		0.0,

		sy,
		-sx * cy,
		cx * cy,
		0.0,

		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	return ret;
}

Matrix44d MatrixHelper::FromRotationX(const Angle& angle)
{
	double c = angle.Cos();
	double s = angle.Sin();

	// data order is defined row major
	// Matrix44d is column major
	// so transpose below before returning
	double data[16] = 
	{
		1.0, 0.0, 0.0, 0.0,
		0.0, c, -s, 0.0,
		0.0, s, c, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromRotationY(const Angle& angle)
{
	double c = angle.Cos();
	double s = angle.Sin();

	// data order is defined row major
	// Matrix44d is column major
	// so transpose below before returning
	double data[16] =
	{
		c, 0.0, s, 0.0,
		0.0, 1.0, 0.0, 0.0,
		-s, 0.0, c, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromRotationZ(const Angle& angle)
{
	double c = angle.Cos();
	double s = angle.Sin();

	// data order is defined row major
	// Matrix44d is column major
	// so transpose below before returning
	double data[16] =
	{
		c, -s, 0.0, 0.0,
		s, c, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromScale(double scale)
{
	return FromScale(scale, scale, scale);
}

Matrix44d MatrixHelper::FromScale(const Vec3d& scale)
{
	return FromScale(scale[0], scale[1], scale[2]);
}

Matrix44d MatrixHelper::FromScale(double scaleX, double scaleY, double scaleZ)
{
	double data[16] = 
	{
		scaleX, 0.0, 0.0, 0.0,
		0.0, scaleY, 0.0, 0.0,
		0.0, 0.0, scaleZ, 0.0,
		0.0, 0.0, 0.0, 1.0,
	};

	Matrix44d ret;
	ret.set(data);
	// dont need to transpose because the matrix is positive definite
	return ret;
}

Matrix44d MatrixHelper::FromTranslation(const Vec3d& translation)
{
	return FromTranslation(translation[0], translation[1], translation[2]);
}

Matrix44d MatrixHelper::FromTranslation(double x, double y, double z)
{
	double data[16] = 
	{
		1.0, 0.0, 0.0, x,
		0.0, 1.0, 0.0, y,
		0.0, 0.0, 1.0, z,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromSkew(const Angle& theta, const Angle& phi)
{
	// from http://faculty.juniata.edu/rhodes/graphics/projectionmat.htm

	double cotTheta = 1.0e6;
	double cotPhi = 1.0e6;

	if (theta.Radians() < gmtl::GMTL_EPSILON && phi.Radians() < gmtl::GMTL_EPSILON)
	{
		cotTheta = 0;
		cotPhi = 0;
	}
	else
	{
		if (Math::abs(Math::tan(theta.Radians())) > gmtl::GMTL_EPSILON)
			cotTheta = 1 / Math::tan(theta.Radians());
		if (Math::abs(Math::tan(phi.Radians())) > gmtl::GMTL_EPSILON)
			cotPhi = 1 / Math::tan(phi.Radians());
	}

	double data[16] = 
	{
		1.0, 0.0, -cotTheta, 0,
		0.0, 1.0, -cotPhi, 0,
		0.0, 0.0, 1.0, 0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromLocalOrientation(const Vec3d& origin, Vec3d xAxis, Vec3d yAxis, Vec3d zAxis)
{
	return FromAxes(xAxis, yAxis, zAxis) * FromTranslation(origin);
}

Matrix44d MatrixHelper::FromViewLookAt(const gmtl::Vec3d& eye, const gmtl::Vec3d& center, const gmtl::Vec3d& up)
{
	Vec3d forward = center - eye;
	Vec3d f = forward;
	normalize(f);

	Vec3d s;
	cross(s, f, up);
	normalize(s);

	Vec3d u;
	cross(u, s, f);
	normalize(u);

	double data[16] =
	{
		s[0], s[1], s[2], 0.0,
		u[0], u[1], u[2], 0.0,
		-f[0], -f[1], -f[2], 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d mAxes;
	mAxes.set(data);
	transpose(mAxes);

	Matrix44d mEye = MatrixHelper::FromTranslation(-eye[0], -eye[1], -eye[2]);
	return mAxes * mEye;
}

Matrix44d MatrixHelper::FromModelLookAt(const gmtl::Vec3d& eye, const gmtl::Vec3d& center, const gmtl::Vec3d& up)
{
	Vec3d forward = center - eye;
	Vec3d f = forward;
	normalize(f);

	Vec3d s;
	cross(s, up, f);
	normalize(s);

	Vec3d u;
	cross(u, f, s);
	normalize(u);

	double data[16] = 
	{
		s[0], u[0], f[0], 0.0,
		s[1], u[1], f[1], 0.0,
		s[2], u[2], f[2], 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d mAxes;
	mAxes.set(data);
	transpose(mAxes);

	Matrix44d mEye = MatrixHelper::FromTranslation(eye[0], eye[1], eye[2]);
	return mEye * mAxes;
}

Matrix44d MatrixHelper::FromPerspective(const Angle& fov, double viewportWidth, double viewportHeight, double near, double far)
{
	double f = 1.0 / fov.TanHalfAngle();
	// We are using *horizontal* field-of-view here. This results in a different matrix than documented in sources
	// using vertical field-of-view.

	double data[16] = 
	{
		f, 0.0, 0.0, 0.0,
		0.0, (f * viewportWidth) / viewportHeight, 0.0, 0.0,
		0.0, 0.0, -(far + near) / (far - near), -(2.0 * far * near) / (far - near),
		0.0, 0.0, -1.0, 0.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromPerspective(double width, double height, double near, double far)
{
	double data[16] = 
	{
		2.0 / width, 0.0, 0.0, 0.0,
		0.0, (2.0 * near) / height, 0.0, 0.0,
		0.0, 0.0, -(far + near) / (far - near), -(2.0 * far * near) / (far - near),
		0.0, 0.0, -1.0, 0.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromOrthographic(double width, double height, double near, double far)
{
	double data[16] = 
	{
		2.0 / width, 0.0, 0.0, 0.0,
		0.0, 2.0 / height, 0.0, 0.0,
		0.0, 0.0, -2.0 / (far - near), -(far + near) / (far - near),
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromOrthographic2D(double width, double height)
{
	double data[16] = 
	{
		2.0 / width, 0.0, 0.0, 0.0,
		0.0, 2.0 / height, 0.0, 0.0,
		0.0, 0.0, -1.0, 0.0,
		0.0, 0.0, 0.0, 1.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromImageToGeographic(int imageWidth, int imageHeight, const Sector& sector)
{
	// Transform from grid coordinates to geographic coordinates. Since the grid is parallel with lines of latitude
	// and longitude, this is a simple scale and translation.

	double sx = sector.DeltaLonDegrees() / imageWidth;
	double sy = -sector.DeltaLatDegrees() / imageHeight;
	double tx = sector.MinLongitude().Degrees();
	double ty = sector.MaxLatitude().Degrees();

	double data[16] =
	{
		sx, 0.0, tx, 0.0,
		0.0, sy, ty, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 0.0
	};

	Matrix44d ret;
	ret.set(data);
	transpose(ret);
	return ret;
}

Matrix44d MatrixHelper::FromGeographicToViewport(const Sector& sector, int x, int y, int width, int height)
{
	Matrix44d transform = MAT_IDENTITY44D;
	transform = transform * MatrixHelper::FromTranslation(-x, -y, 0.0);
	transform = transform * MatrixHelper::FromScale(width / sector.DeltaLonDegrees(), height / sector.DeltaLatDegrees(), 1.0);
	transform = transform * MatrixHelper::FromTranslation(-sector.MinLongitude().Degrees(), -sector.MinLatitude().Degrees(), 0.0);
	return transform;
}

Matrix44d MatrixHelper::FromViewportToGeographic(const Sector& sector, int x, int y, int width, int height)
{
	Matrix44d transform = MAT_IDENTITY44D;
	transform = transform * MatrixHelper::FromTranslation(sector.MinLongitude().Degrees(), sector.MinLatitude().Degrees(), 0.0);
	transform = transform * MatrixHelper::FromScale(sector.DeltaLonDegrees() / width, sector.DeltaLatDegrees() / height, 1.0);
	transform = transform * MatrixHelper::FromTranslation(x, y, 0.0);
	return transform;
}


Angle MatrixHelper::GetRotationX(const gmtl::Matrix44d& mat)
{
	double yRadians = Math::aSin(mat[2][0] /* this.m13 */);
	double cosY = Math::cos(yRadians);
	
	//if (MathHelper::IsZero(cosY))
	//	return null;

	double xRadians;
	// No Gimball lock.
	if (Math::abs(cosY) > 0.005)
	{
		xRadians = Math::aTan2(-mat[2][1] /*this.m23*/ / cosY, mat[2][2] /*this.m33*/ / cosY);
	}
	// Gimball lock has occurred. Rotation around X axis becomes rotation around Z axis.
	else
	{
		xRadians = 0;
	}

	//if (MathHelper::IsNan(xRadians))
	//	return null;

	return Angle::FromRadians(xRadians);
}

Angle MatrixHelper::GetRotationY(const Matrix44d& mat)
{
	double yRadians = Math::aSin(mat[2][0] /*this.m13*/);
	//if (Double.isNaN(yRadians))
	//	return null;

	return Angle::FromRadians(yRadians);
}

Angle MatrixHelper::GetRotationZ(const Matrix44d& mat)
{
	double yRadians = Math::aSin(mat[2][0] /*this.m13*/);
	double cosY = Math::cos(yRadians);
	
	//if (isZero(cosY))
	//	return null;

	double zRadians;
	// No Gimball lock.
	if (Math::abs(cosY) > 0.005)
	{
		zRadians = Math::aTan2(-mat[1][0] /*this.m12*/ / cosY, mat[0][0] /*this.m11*/ / cosY);
	}
	// Gimball lock has occurred. Rotation around X axis becomes rotation around Z axis.
	else
	{
		zRadians = Math::aTan2(mat[0][1] /*this.m21*/, mat[1][1] /*this.m22*/);
	}

	//if (Double.isNaN(zRadians))
	//	return null;

	return Angle::FromRadians(zRadians);
}

Angle MatrixHelper::GetKMLRotationX(const Matrix44d& mat)    // KML assumes the order of rotations is YXZ, positive CW
{
	double xRadians = Math::aSin(-mat[2][1] /*this.m23*/);
	
	//if (Double.isNaN(xRadians))
	//	return null;

	return Angle::FromRadians(-xRadians);    // negate to make angle CW
}

Angle MatrixHelper::GetKMLRotationY(const Matrix44d& mat)    // KML assumes the order of rotations is YXZ, positive CW
{
	double xRadians = Math::aSin(-mat[2][1] /*this.m23*/);
	
	//if (Double.isNaN(xRadians))
	//	return null;

	double yRadians;
	if (xRadians < Math::PI / 2)
	{
		if (xRadians > -Math::PI / 2)
		{
			yRadians = Math::aTan2(mat[2][0] /*this.m13*/, mat[2][2] /*this.m33*/);
		}
		else
		{
			yRadians = -Math::aTan2(-mat[1][0] /*this.m12*/, mat[0][0] /*this.m11*/);
		}
	}
	else
	{
		yRadians = Math::aTan2(-mat[1][0] /*this.m12*/, mat[0][0] /*this.m11*/);
	}

	//if (Double.isNaN(yRadians))
	//	return null;

	return Angle::FromRadians(-yRadians);    // negate angle to make it CW
}

Angle MatrixHelper::GetKMLRotationZ(const Matrix44d& mat)    // KML assumes the order of rotations is YXZ, positive CW
{
	double xRadians = Math::aSin(-mat[2][1] /*this.m23*/);
	
	//if (Double.isNaN(xRadians))
	//	return null;

	double zRadians;
	if (xRadians < Math::PI / 2 && xRadians > -Math::PI / 2)
	{
		zRadians = Math::aTan2(mat[0][1] /*this.m21*/, mat[1][1] /*this.m22*/);
	}
	else
	{
		zRadians = 0;
	}

	//if (Double.isNaN(zRadians))
	//	return null;

	return Angle::FromRadians(-zRadians);    // negate angle to make it CW
}