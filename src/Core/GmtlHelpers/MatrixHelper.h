#ifndef CORE_GMTL_HELPERS_MATRIXHELPER_H
#define CORE_GMTL_HELPERS_MATRIXHELPER_H

#include "..\Core.h"
#include "..\Geometry\Angle.h"
#include "..\Geometry\Sector.h"

class MatrixHelper 
{
public:

	/**
	* Returns a Cartesian transform <code>Matrix</code> that maps a local orientation to model coordinates. The
	* orientation is specified by an array of three <code>axes</code>. The <code>axes</code> array must contain three
	* non-null vectors, which are interpreted in the following order: x-axis, y-axis, z-axis. This ensures that the
	* axes in the returned <code>Matrix</code> have unit length and are orthogonal to each other.
	*
	* @param axes an array must of three non-null vectors defining a local orientation in the following order: x-axis,
	*             y-axis, z-axis.
	*
	* @return a <code>Matrix</code> that a transforms local coordinates to world coordinates.
	*
	* @throws IllegalArgumentException if <code>axes</code> is <code>null</code>, if <code>axes</code> contains less
	*                                  than three elements, or if any of the first three elements in <code>axes</code>
	*                                  is <code>null</code>.
	*/
	static gmtl::Matrix44d				FromAxes(gmtl::Vec3d xAxis, gmtl::Vec3d yAxis, gmtl::Vec3d zAxis);

	static gmtl::Matrix44d				FromAxisAngle(const Angle& angle, const gmtl::Vec3d& axis);
	static gmtl::Matrix44d				FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ);

	static gmtl::Matrix44d				FromQuaternion(const gmtl::Quatd& quaternion);

	static gmtl::Matrix44d				FromRotationXYZ(const Angle& xRotation, const Angle& yRotation, const Angle& zRotation);
	static gmtl::Matrix44d				FromRotationX(const Angle& angle);
	static gmtl::Matrix44d				FromRotationY(const Angle& angle);
	static gmtl::Matrix44d				FromRotationZ(const Angle& angle);

	static gmtl::Matrix44d				FromScale(double scale);
	static gmtl::Matrix44d				FromScale(const gmtl::Vec3d& scale);
	static gmtl::Matrix44d				FromScale(double scaleX, double scaleY, double scaleZ);
	
	static gmtl::Matrix44d				FromTranslation(const gmtl::Vec3d& translation);
	static gmtl::Matrix44d				FromTranslation(double x, double y, double z);

	static gmtl::Matrix44d				FromSkew(const Angle& theta, const Angle& phi);

	/**
	* Returns a Cartesian transform <code>Matrix</code> that maps a local origin and orientation to model coordinates.
	* The transform is specified by a local <code>origin</code> and an array of three <code>axes</code>. The
	* <code>axes</code> array must contain three non-null vectors, which are interpreted in the following order:
	* x-axis, y-axis, z-axis. This ensures that the axes in the returned <code>Matrix</code> have unit length and are
	* orthogonal to each other.
	*
	* @param origin the origin of the local coordinate system.
	* @param axes   an array must of three non-null vectors defining a local orientation in the following order:
	*               x-axis, y-axis, z-axis.
	*
	* @return a <code>Matrix</code> that transforms local coordinates to world coordinates.
	*
	* @throws IllegalArgumentException if <code>origin</code> is <code>null</code>, if <code>axes</code> is
	*                                  <code>null</code>, if <code>axes</code> contains less than three elements, or if
	*                                  any of the first three elements in <code>axes</code> is <code>null</code>.
	*/
	static gmtl::Matrix44d				FromLocalOrientation(const gmtl::Vec3d& origin, gmtl::Vec3d xAxis, gmtl::Vec3d yAxis, gmtl::Vec3d zAxis);

	/**
	* Returns a viewing matrix in model coordinates defined by the specified View eye point, reference point indicating
	* the center of the scene, and up vector. The eye point, center point, and up vector are in model coordinates. The
	* returned viewing matrix maps the reference center point to the negative Z axis, and the eye point to the origin,
	* and the up vector to the positive Y axis. When this matrix is used to define an OGL viewing transform along with
	* a typical projection matrix such as {@link #fromPerspective(Angle, double, double, double, double)} , this maps
	* the center of the scene to the center of the viewport, and maps the up vector to the viewoport's positive Y axis
	* (the up vector points up in the viewport). The eye point and reference center point must not be coincident, and
	* the up vector must not be parallel to the line of sight (the vector from the eye point to the reference center
	* point).
	*
	* @param eye    the eye point, in model coordinates.
	* @param center the scene's reference center point, in model coordinates.
	* @param up     the direction of the up vector, in model coordinates.
	*
	* @return a viewing matrix in model coordinates defined by the specified eye point, reference center point, and up
	*         vector.
	*
	* @throws IllegalArgumentException if any of the eye point, reference center point, or up vector are null, if the
	*                                  eye point and reference center point are coincident, or if the up vector and the
	*                                  line of sight are parallel.
	*/
	static gmtl::Matrix44d				FromViewLookAt(const gmtl::Vec3d& eye, const gmtl::Vec3d& center, const gmtl::Vec3d& up);

	/**
	* Returns a local origin transform matrix in model coordinates defined by the specified eye point, reference point
	* indicating the center of the local scene, and up vector. The eye point, center point, and up vector are in model
	* coordinates. The returned viewing matrix maps the the positive Z axis to the reference center point, the origin
	* to the eye point, and the positive Y axis to the up vector. The eye point and reference center point must not be
	* coincident, and the up vector must not be parallel to the line of sight (the vector from the eye point to the
	* reference center point).
	*
	* @param eye    the eye point, in model coordinates.
	* @param center the scene's reference center point, in model coordinates.
	* @param up     the direction of the up vector, in model coordinates.
	*
	* @return a viewing matrix in model coordinates defined by the specified eye point, reference center point, and up
	*         vector.
	*
	* @throws IllegalArgumentException if any of the eye point, reference center point, or up vector are null, if the
	*                                  eye point and reference center point are coincident, or if the up vector and the
	*                                  line of sight are parallel.
	*/
	static gmtl::Matrix44d				FromModelLookAt(const gmtl::Vec3d& eye, const gmtl::Vec3d& center, const gmtl::Vec3d& up);

	static gmtl::Matrix44d				FromPerspective(const Angle& fov, double viewportWidth, double viewportHeight, double near, double far);
	static gmtl::Matrix44d				FromPerspective(double width, double height, double near, double far);
	static gmtl::Matrix44d				FromOrthographic(double width, double height, double near, double far);
	static gmtl::Matrix44d				FromOrthographic2D(double width, double height);

	/**
	* Computes a <code>Matrix</code> that will map a aligned 2D grid coordinates to geographic coordinates in degrees.
	* It is assumed that the destination grid is parallel with lines of latitude and longitude, and has its origin in
	* the upper left hand corner.
	*
	* @param sector      the grid sector.
	* @param imageWidth  the grid width.
	* @param imageHeight the grid height.
	*
	* @return <code>Matrix</code> that will map from grid coordinates to geographic coordinates in degrees.
	*
	* @throws IllegalArgumentException if <code>sector</code> is null, or if either <code>width</code> or
	*                                  <code>height</code> are less than 1.
	*/
	static gmtl::Matrix44d				FromImageToGeographic(int imageWidth, int imageHeight, const Sector& sector);
	static gmtl::Matrix44d				FromGeographicToViewport(const Sector& sector, int x, int y, int width, int height);
	static gmtl::Matrix44d				FromViewportToGeographic(const Sector& sector, int x, int y, int width, int height);

	static Angle						GetRotationX(const gmtl::Matrix44d& mat);
	static Angle						GetRotationY(const gmtl::Matrix44d& mat);
	static Angle						GetRotationZ(const gmtl::Matrix44d& mat);
	static gmtl::Vec3d					GetTranslation(const gmtl::Matrix44d& mat);

	static Angle						GetKMLRotationX(const gmtl::Matrix44d& mat);
	static Angle						GetKMLRotationY(const gmtl::Matrix44d& mat);
	static Angle						GetKMLRotationZ(const gmtl::Matrix44d& mat);

private:

	MatrixHelper();

	static gmtl::Matrix44d				FromAxisAngle(const Angle& angle, double axisX, double axisY, double axisZ, bool normalize);
	static gmtl::Matrix44d				FromQuaternion(double x, double y, double z, double w, bool normalize);

};

#endif /* CORE_GMTL_HELPERS_MATRIXHELPER_H */