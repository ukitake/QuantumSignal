#ifndef CORE_GEOMETRY_FRUSTUM_H
#define CORE_GEOMETRY_FRUSTUM_H

#include "..\Core.h"
#include "Plane.h"
#include "Extent.h"

class Frustum
{
	typedef gmtl::Vec3d Vec3;

protected:
	Plane						left;
	Plane						right;
	Plane						bottom;
	Plane						top;
	Plane						near;
	Plane						far;

public:

	static Frustum				FromProjectionMatrix(const gmtl::Matrix44d& projectionMatrix);
	static Frustum				FromPerspective(const Angle& horizontalFieldOfView, int viewportWidth, int viewportHeight, double near, double far);
	static Frustum				FromPerspective(double width, double height, double near, double far);
	static Frustum				FromPerspectiveVecs(const Vec3& vTL, const Vec3& vTR, const Vec3& vBL, const Vec3& vBR, double near, double far);


	Frustum() : left(Plane(1, 0, 0, 1)), right(Plane(-1, 0, 0, 1)), bottom(Plane(0, 1, 0, 1)), top(Plane(0, -1, 0, 1)), near(Plane(0, 0, -1, 1)), far(Plane(0, 0, 1, 1)) { }
	Frustum(const Plane& left, const Plane& right, const Plane& bottom, const Plane& top, const Plane& near, const Plane& far);

	Plane						Left() const { return left; }
	Plane						Right() const { return right; }
	Plane						Bottom() const { return bottom; }
	Plane						Top() const { return top; }
	Plane						Near() const { return near; }
	Plane						Far() const { return far; }
	void						AllPlanes(Plane arr[]) const;

	bool						Intersects(const Extent& extent) const;
	bool						IntersectsSegment(const Vec3& pa, const Vec3& pb) const;
	bool						Contains(const Extent& extent) const;
	bool						Contains(const Vec3& point) const;

	Frustum						TransformBy(const gmtl::Matrix44d& matrix) const;
	void						GetCorners(Vec3 arr[]) const;

	bool						operator==(const Frustum& that) const;
	bool						operator!=(const Frustum& that) const;
};

#endif /* CORE_GEOMETRY_FRUSTUM_H */