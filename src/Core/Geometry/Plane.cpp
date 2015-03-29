#include "Plane.h"
#include "..\GmtlHelpers\MathHelper.h"

Plane Plane::FromPoints(const Vec3& pa, const Vec3& pb, const Vec3& pc)
{
	Vec3 vab = pb- pa;
	Vec3 vac = pc - pa;
	Vec3 _n;
	cross(_n, vab, vac);
	double d = -gmtl::dot(_n, pa);

	return Plane(_n[0], _n[1], _n[2], d);
}

Plane Plane::Normalize() const
{
	double length = gmtl::length(n);
	if (length == 0.0) // should not happen, but check to be sure.
		return *this;

	return Plane(Vec3(
		n[0] / length,
		n[1] / length,
		n[2] / length),
		1.0);
}

double Plane::Dot(const gmtl::Vec4d& p) const
{
	return n[0] * p[0] + n[1] * p[1] + n[2] * p[2] + d * p[3];
}

gmtl::Vec3d Plane::Intersect(const Line& line) const
{
	double t = IntersectDistance(line);

	if (MathHelper::IsNan(t))
		return Vec3();

	if (MathHelper::IsInf(t))
		return line.Origin();

	return line.GetPointAt(t);
}

double Plane::IntersectDistance(const Line& line) const
{
	double ldotv = dot(n, line.Direction());
	if (ldotv == 0) // are line and plane parallel
	{
		double ldots = dot(n, line.Origin());
		if (ldots == 0)
			return DBL_MAX; // line is coincident with the plane
		else
			return NAN; // line is not coincident with the plane
	}

	return -dot(n, line.Origin()) / ldotv; // ldots / ldotv
}

gmtl::Vec3d Plane::Intersect(const Vec3& pa, const Vec3& pb) const
{
	// Test if line segment is in fact a point
	if (pa == pb)
	{
		double d = DistanceTo(pa);
		if (d == 0)
			return pa;
		else
			return Vec3();
	}

	Line l = Line::FromSegment(pa, pb);
	double t = IntersectDistance(l);

	if (MathHelper::IsInf(t))
		return Vec3(DBL_MAX, DBL_MAX, DBL_MAX);

	if (MathHelper::IsNan(t) || t < 0 || t > 1)
		return Vec3();

	return l.GetPointAt(t);
}

bool Plane::Clip(const Vec3& pa, const Vec3& pb, Vec3 arr[]) const
{
	if (pa == pb)
		return false;

	// Get the projection of the segment onto the plane.
	Line line = Line::FromSegment(pa, pb);
	double ldotv = dot(n, line.Direction());

	// Are the line and plane parallel?
	if (ldotv == 0) // line and plane are parallel and maybe coincident
	{
		double ldots = dot(n, line.Origin());
		if (ldots == 0){
			arr[0] = pa;
			arr[1] = pb;
			return true; // line is coincident with the plane
		}
		else
			return false; // line is not coincident with the plane
	}

	// Not parallel so the line intersects. But does the segment intersect?
	double t = -dot(n, line.Origin()) / ldotv; // ldots / ldotv
	if (t < 0 || t > 1) // segment does not intersect
		return false;

	Vec3 p = line.GetPointAt(t);
	if (ldotv > 0){
		arr[0] = p;
		arr[1] = pb;
		return true;
	}
	else {
		arr[0] = pa;
		arr[1] = p;
		return true;
	}
}

double Plane::DistanceTo(const Vec3& p) const
{
	return dot(n, (p - (n * d)));
}

int Plane::OnSameSide(const Vec3& pa, const Vec3& pb) const
{
	double da = DistanceTo(pa);
	double db = DistanceTo(pb);

	if (da < 0 && db < 0)
		return -1;

	if (da > 0 && db > 0)
		return 1;

	return 0;
}

int Plane::OnSameSide(const std::vector<Vec3>& pts) const
{
	double d = DistanceTo(pts[0]);
	int side = d < 0 ? -1 : d > 0 ? 1 : 0;
	if (side == 0)
		return 0;

	for (int i = 1; i < pts.size(); i++)
	{
		d = DistanceTo(pts[i]);
		if ((side == -1 && d < 0) || (side == 1 && d > 0))
			continue;

		return 0; // point is not on same side as the others
	}

	return side;
}

gmtl::Vec3d Plane::Intersect(const Plane& pa, const Plane& pb, const Plane& pc)
{
	Vec3 na = pa.Normal();
	Vec3 nb = pb.Normal();
	Vec3 nc = pc.Normal();

	double data[9] = 
	{
		na[0], na[1], na[2],
		nb[0], nb[1], nb[2],
		nc[0], nc[1], nc[2]
	};

	gmtl::Matrix33d matrix;
	matrix.set(data);
	transpose(matrix);

	gmtl::Matrix33d mInverse = gmtl::invert(matrix);

	Vec3 D = Vec3(-pa.Distance(), -pb.Distance(), -pc.Distance());

	return mInverse * D;
}


