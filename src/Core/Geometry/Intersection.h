#ifndef CORE_GEOMETRY_INTERSECTION_H
#define CORE_GEOMETRY_INTERSECTION_H

#include "..\Core.h"
#include "Position.h"

class Intersection
{
	typedef gmtl::Vec3d	Vec3;

protected:
	Vec3					intersectionPoint;
	Position				intersectionPosition;
	bool					isTangent;

public:
	Intersection(const Vec3& iP, bool iT) : intersectionPoint(iP), isTangent(iT) { }
	Intersection(const Vec3& iP, const Position& pos, bool iT) : intersectionPoint(iP), intersectionPosition(pos), isTangent(iT) { }

	Position				getIntersectionPosition() const { return intersectionPosition; }
	void					setIntersectionPosition(const Position& pos) { intersectionPosition = pos; }

	Vec3					getIntersectionPoint() const { return intersectionPoint; }
	void					setIntersectionPoint(const Vec3& point) { intersectionPoint = point; }

	bool					getIsTangent() const { return isTangent; }
	void					setIsTangent(bool iT) { isTangent = iT; }

	bool					operator==(const Intersection& that) const { return intersectionPoint == that.intersectionPoint && intersectionPosition == that.intersectionPosition && isTangent == that.isTangent; }
	bool					operator!=(const Intersection& that) const { return !((*this) == that); }
};

#endif /* CORE_GEOMETRY_INTERSECTION_H */