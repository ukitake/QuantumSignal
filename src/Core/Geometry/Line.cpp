#include "Line.h"

typedef gmtl::Vec3d Vec3;

Line Line::FromSegment(const Vec3& pa, const Vec3& pb)
{
	return Line(pa, Vec3(pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]));
}

Vec3 Line::NearestPointOnSegment(const Vec3& p0, const Vec3& p1, const Vec3& p)
{
	Vec3 v = p1 - p0;
	Vec3 w = p - p0;

	double c1 = dot(w, v);
	double c2 = dot(v, v);

	if (c1 <= 0)
		return p0;
	if (c2 <= c1)
		return p1;

	return p0 + (v * (c1 / c2));
}

double Line::DistanceToSegment(const Vec3& p0, const Vec3& p1, const Vec3& p)
{
	Vec3 pb = NearestPointOnSegment(p0, p1, p);
	Vec3 line = p - pb;
	return length(line);
}

bool Line::IsPointBehindLineOrigin(const Vec3& point)
{
	return gmtl::dot(point - origin, direction) < 0.0;
}

Vec3 Line::NearestIntersectionPoint(const std::vector<Intersection>& intersections)
{
	Vec3 intersectionPoint;

	// Find the nearest intersection that's in front of the ray origin.
	double nearestDistance = DBL_MAX;
	for (Intersection intersection : intersections)
	{
		// Ignore any intersections behind the line origin.
		if (!this->IsPointBehindLineOrigin(intersection.getIntersectionPoint()))
		{
			Vec3 line = intersection.getIntersectionPoint() - origin;
			double d = gmtl::length(line);
			if (d < nearestDistance)
			{
				intersectionPoint = intersection.getIntersectionPoint();
				nearestDistance = d;
			}
		}
	}

	return intersectionPoint;
}

Vec3 Line::NearestPointTo(const Vec3& p)
{
	Vec3 w = p - origin;

	double c1 = dot(w, direction);
	double c2 = dot(direction, direction);

	return origin + (direction * (c1 / c2));
}

double Line::DistanceTo(const Vec3& p)
{
	Vec3 line = p - this->NearestPointTo(p);
	return length(line);
}