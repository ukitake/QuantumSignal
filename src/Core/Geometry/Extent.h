#ifndef CORE_GEOMETRY_EXTENT_H
#define CORE_GEOMETRY_EXTENT_H

#include "..\Core.h"
#include "Frustum.h"
#include "Line.h"
#include "Plane.h"
#include "Intersection.h"

class Frustum;

class Extent
{
public:
	virtual								~Extent() { }

	virtual gmtl::Vec3d					Center() const = 0;
	virtual double						Diameter() const = 0;
	virtual double						Radius() const = 0;
	virtual bool						Intersects(const Frustum& frustum) const = 0;
	virtual std::vector<Intersection>	Intersect(const Line& line) const = 0;
	virtual bool						Intersects(const Line& line) const = 0;
	virtual bool						Intersects(const Plane& plane) const = 0;
	virtual double						GetEffectiveRadius(const Plane& plane) = 0;

private:
	Extent() {}
};

#endif /* CORE_GEOMETRY_EXTENT_H */