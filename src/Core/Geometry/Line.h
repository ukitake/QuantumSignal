#ifndef CORE_GEOMETRY_LINE_H
#define CORE_GEOMETRY_LINE_H

#include "..\Core.h"
#include "Intersection.h"

class Line 
{
	typedef gmtl::Vec3d		Vec3;

private:
	Vec3					origin;
	Vec3					direction;

	Line() : origin(Vec3()), direction(Vec3()) { }
public:

	static Line				FromSegment(const Vec3& pa, const Vec3& pb);
	static Vec3				NearestPointOnSegment(const Vec3& p0, const Vec3& p1, const Vec3& p);
	static double			DistanceToSegment(const Vec3& p0, const Vec3& p1, const Vec3& p);

	Line(const Vec3& o, const Vec3& d) : origin(o), direction(d) { }

	Vec3					Direction() const { return direction; }
	Vec3					Origin() const { return origin; }
	Vec3					GetPointAt(double t) const { Vec3 n(direction); normalize(n); return origin + (direction * t); }
	double					SelfDot() const { return dot(origin, direction); }

	Vec3					NearestPointTo(const Vec3& p);
	double					DistanceTo(const Vec3& p);
	
	bool					IsPointBehindLineOrigin(const Vec3& point);
	Vec3					NearestIntersectionPoint(const std::vector<Intersection>& intersections);

	bool					operator==(const Line& that) const { return origin == that.origin && direction == that.direction; }
	bool					operator!=(const Line& that) const { return origin != that.origin || direction != that.direction; }

};

#endif /* CORE_GEOMETRY_LINE_H */