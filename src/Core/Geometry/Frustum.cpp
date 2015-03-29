#include "Frustum.h"

Frustum::Frustum(const Plane& left, const Plane& right, const Plane& bottom, const Plane& top, const Plane& near, const Plane& far)
{
	this->left = left;
	this->right = right;
	this->bottom = bottom;
	this->top = top;
	this->near = near;
	this->far = far;
}

void Frustum::AllPlanes(Plane arr[]) const
{
	arr[0] = left;
	arr[1] = right;
	arr[2] = bottom;
	arr[3] = top;
	arr[4] = near;
	arr[5] = far;
}

bool Frustum::operator==(const Frustum& that) const
{
	return left == that.left
		&& right == that.right
		&& bottom == that.bottom
		&& top == that.top
		&& near == that.near
		&& far == that.far;
}

bool Frustum::operator!=(const Frustum& that) const
{
	return left != that.left
		|| right != that.right
		|| bottom != that.bottom
		|| top != that.top
		|| near != that.near
		|| far != that.far;
}

Frustum Frustum::FromProjectionMatrix(const gmtl::Matrix44d& matrix)
{
	gmtl::Matrix44d m = matrix;

	// Extract the six clipping planes from the projection-matrix.
	Plane leftPlane = Plane(m[0][3] + m[0][0], m[1][3] + m[1][0], m[2][3] + m[2][0], m[3][3] + m[3][0]).Normalize();
	Plane rightPlane = Plane(m[0][3] - m[0][0], m[1][3] - m[1][0], m[2][3] - m[2][0], m[3][3] - m[3][0]).Normalize();
	Plane bottomPlane = Plane(m[0][3] + m[0][1], m[1][3] + m[1][1], m[2][3] + m[2][1], m[3][3] + m[3][1]).Normalize();
	Plane topPlane = Plane(m[0][3] - m[0][1], m[1][3] - m[1][1], m[2][3] - m[2][1], m[3][3] - m[3][1]).Normalize();
	Plane nearPlane = Plane(m[0][3] + m[0][2], m[1][3] + m[1][2], m[2][3] + m[2][2], m[3][3] + m[3][2]).Normalize();
	Plane farPlane = Plane(m[0][3] - m[0][2], m[1][3] - m[1][2], m[2][3] - m[2][2], m[3][3] - m[3][2]).Normalize();

	return Frustum(leftPlane, rightPlane, bottomPlane, topPlane, nearPlane, farPlane);
}

Frustum Frustum::FromPerspective(const Angle& horizontalFieldOfView, int viewportWidth, int viewportHeight, double near, double far)
{
	double fov = horizontalFieldOfView.Degrees();
	double farMinusNear = far - near;

	double focalLength = 1.0 / horizontalFieldOfView.TanHalfAngle();
	double aspect = viewportHeight / (double)viewportWidth;
	double lrLen = gmtl::Math::sqrt(focalLength * focalLength + 1);
	double btLen = gmtl::Math::sqrt(focalLength * focalLength + aspect * aspect);
	Plane leftPlane = Plane(focalLength / lrLen, 0.0, 0.0 - 1.0 / lrLen, 0);
	Plane rightPlane = Plane(0.0 - focalLength / lrLen, 0.0, 0.0 - 1.0 / lrLen, 0.0);
	Plane bottomPlane = Plane(0.0, focalLength / btLen, 0.0 - aspect / btLen, 0.0);
	Plane topPlane = Plane(0.0, 0.0 - focalLength / btLen, 0.0 - aspect / btLen, 0.0);
	Plane nearPlane = Plane(0.0, 0.0, 0.0 - 1.0, 0.0 - near);
	Plane farPlane = Plane(0.0, 0.0, 1.0, far);
	return Frustum(leftPlane, rightPlane, bottomPlane, topPlane, nearPlane, farPlane);
}

Frustum Frustum::FromPerspective(double width, double height, double near, double far)
{
	double farMinusNear = far - near;

	double width_over_2 = width / 2.0;
	double height_over_2 = height / 2.0;
	Plane leftPlane = Plane(1.0, 0.0, 0.0, width_over_2);
	Plane rightPlane = Plane(-1.0, 0.0, 0.0, width_over_2);
	Plane bottomPlane = Plane(0.0, 1.0, 0.0, height_over_2);
	Plane topPlane = Plane(0.0, -1.0, 0.0, height_over_2);
	Plane nearPlane = Plane(0.0, 0.0, -1.0, (near < 0.0) ? near : -near);
	Plane farPlane = Plane(0.0, 0.0, 1.0, (far < 0.0) ? -far : far);
	return Frustum(leftPlane, rightPlane, bottomPlane, topPlane, nearPlane, farPlane);
}

Frustum Frustum::FromPerspectiveVecs(const Vec3& vTL, const Vec3& vTR, const Vec3& vBL, const Vec3& vBR, double near, double far)
{
	double farMinusNear = far - near;

	Vec3 lpn;
	cross(lpn, vBL, vTL);
	normalize(lpn);
	Plane leftPlane = Plane(lpn[0], lpn[1], lpn[2], 0);
	
	Vec3 rpn;
	cross(rpn, vTR, vBR);
	normalize(rpn);
	Plane rightPlane = Plane(rpn[0], rpn[1], rpn[2], 0);
	Vec3 bpn;
	cross(bpn, vBR, vBL);
	normalize(bpn);
	Plane bottomPlane = Plane(bpn[0], bpn[1], bpn[2], 0);
	Vec3 tpn;
	cross(tpn, vTL, vTR);
	normalize(tpn);
	Plane topPlane = Plane(tpn[0], tpn[1], tpn[2], 0);

	Plane nearPlane = Plane(0.0, 0.0, 0.0 - 1.0, 0.0 - near);
	Plane farPlane = Plane(0.0, 0.0, 1.0, far);
	return Frustum(leftPlane, rightPlane, bottomPlane, topPlane, nearPlane, farPlane);
}

bool Frustum::Intersects(const Extent& extent) const
{
	return extent.Intersects(*this);
}

bool Frustum::IntersectsSegment(const Vec3& pa, const Vec3& pb) const
{
	// First do a trivial accept test.
	if (Contains(pa) || Contains(pb))
		return true;

	if (pa == pb)
		return false;

	Plane planes[6];
	AllPlanes(planes);
	for (int i = 0; i < 6; i++)
	{
		Plane p = planes[i];

		// See if both points are behind the plane and therefore not in the frustum.
		if (p.OnSameSide(pa, pb) < 0)
			return false;

		// See if the segment intersects the plane.
		Vec3 points[2];
		if (p.Clip(pa, pb, points))
			return true;
	}

	return false; // segment does not intersect frustum
}

bool Frustum::Contains(const Extent& extent) const
{
	// TODO: This method should be implemented in the concrete extent classes and those implementing methods
	// invoked here, as is done above for intersects(Frustum).

	// See if the extent's bounding sphere is entirely within the frustum. The dot product of the extent's center
	// point with each plane's vector provides a distance to each plane. If this distance is less than the extent's
	// radius, some part of the extent is clipped by that plane and therefore is not completely contained in the
	// space enclosed by this Frustum.

	Vec3 _c = extent.Center();
	gmtl::Vec4d c = gmtl::Vec4d(_c[0], _c[1], _c[2], 0);
	double r = extent.Radius();

	if (far.Dot(c) <= r)
		return false;
	if (left.Dot(c) <= r)
		return false;
	if (right.Dot(c) <= r)
		return false;
	if (top.Dot(c) <= r)
		return false;
	if (bottom.Dot(c) <= r)
		return false;
	//noinspection RedundantIfStatement
	if (near.Dot(c) <= r)
		return false;

	return true;
}

bool Frustum::Contains(const Vec3& p) const
{
	// See if the point is entirely within the frustum. The dot product of the point with each plane's vector
	// provides a distance to each plane. If this distance is less than 0, the point is clipped by that plane and
	// neither intersects nor is contained by the space enclosed by this Frustum.
	
	gmtl::Vec4d point = gmtl::Vec4d(p[0], p[1], p[2], 0);

	if (far.Dot(point) <= 0)
		return false;
	if (left.Dot(point) <= 0)
		return false;
	if (right.Dot(point) <= 0)
		return false;
	if (top.Dot(point) <= 0)
		return false;
	if (bottom.Dot(point) <= 0)
		return false;
	//noinspection RedundantIfStatement
	if (near.Dot(point) <= 0)
		return false;

	return true;
}

Frustum Frustum::TransformBy(const gmtl::Matrix44d& matrix) const
{
	Plane left = Plane(matrix * left.Vector());
	Plane right = Plane(matrix * right.Vector());
	Plane bottom = Plane(matrix * bottom.Vector());
	Plane top = Plane(matrix * top.Vector());
	Plane near = Plane(matrix * near.Vector());
	Plane far = Plane(matrix * far.Vector());
	return Frustum(left, right, bottom, top, near, far);
}

void Frustum::GetCorners(Vec3 arr[]) const
{
	arr[0] = Plane::Intersect(near, bottom, left);
	arr[1] = Plane::Intersect(near, bottom, right);
	arr[2] = Plane::Intersect(near, top, left);
	arr[3] = Plane::Intersect(near, top, right);

	arr[4] = Plane::Intersect(far, bottom, left);
	arr[5] = Plane::Intersect(far, bottom, right);
	arr[6] = Plane::Intersect(far, top, left);
	arr[7] = Plane::Intersect(far, top, right);
}