#ifndef CORE_GEOMETRY_PLANE_H
#define CORE_GEOMETRY_PLANE_H

#include "..\Core.h"
#include "Line.h"

class Plane
{
	typedef gmtl::Vec3d Vec3;

private:
	Vec3					n;
	double					d;

public:

	Plane();
	Plane(const gmtl::Vec4d vec) : n(Vec3(vec[0], vec[1], vec[2])), d(vec[3]) { }
	Plane(const Vec3& vec, double _d) : n(vec), d(_d) { normalize(n); }
	Plane(double nx, double ny, double nz, double _d) : n(Vec3(nx, ny, nz)), d(_d) { normalize(n); }

	static Plane			FromPoints(const Vec3& pa, const Vec3& pb, const Vec3& pc);
	static Vec3				Intersect(const Plane& pa, const Plane& pb, const Plane& pc);

	Vec3					Normal() const { return n; }
	double					Distance() const { return d; }
	gmtl::Vec4d				Vector() const { return gmtl::Vec4d(n[0], n[1], n[2], d); }
	Plane					Normalize() const;

	double					Dot(const gmtl::Vec4d& p) const;
	Vec3					Intersect(const Line& line) const;
	double					IntersectDistance(const Line& line) const;
	Vec3					Intersect(const Vec3& pa, const Vec3& pb) const;
	bool					Clip(const Vec3& pa, const Vec3& pb, Vec3 arr[]) const;
	double					DistanceTo(const Vec3& p) const;
	int						OnSameSide(const Vec3& pa, const Vec3& pb) const;
	int						OnSameSide(const std::vector<Vec3>& pts) const;

	bool					operator==(const Plane& that) const { return n == that.n && d == that.d; }
	bool					operator!=(const Plane& that) const { return !((*this) == that); }
};

#endif /* CORE_GEOMETRY_PLANE_H */