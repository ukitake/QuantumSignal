#ifndef CORE_GEOMETRY_SECTOR_H
#define CORE_GEOMETRY_SECTOR_H

#include "..\Core.h"
#include "LatLon.h"
#include "Angle.h"

class Sector
{
public:
	static Sector				FromDegrees(double minLatitude, double maxLatitude, double minLongitude, double maxLongitude);
	static Sector				FromRadians(double minLatitude, double maxLatitude, double minLongitude, double maxLongitude);

	static Sector				BoundingSector(const std::vector<LatLon>& locations);
	static Sector				BoundingSector(const LatLon& pA, const LatLon& pB);
	static std::vector<Sector>	SplitBoundingSectors(const std::vector<LatLon>& locations);

	static Sector				Union(const Sector& sectorA, const Sector& sectorB);
	static Sector				Union(const std::vector<Sector>& sectors);

	Sector();
	Sector(const Angle& minLatitude, const Angle& maxLatitude, const Angle& minLongitude, const Angle& maxLongitude);
	Sector(const Sector& sector);

	Angle						MinLatitude() const { return minLatitude; }
	Angle						MaxLatitude() const { return maxLatitude; }
	Angle						MinLongitude() const { return minLongitude; }
	Angle						MaxLongitude() const { return maxLongitude; }
	Angle						DeltaLat() const { return deltaLat; }
	Angle						DeltaLon() const { return deltaLon; }
	double						DeltaLatDegrees() const { return deltaLat.Degrees(); }
	double						DeltaLonDegrees() const { return deltaLon.Degrees(); }

	bool						IsWithinLatLonLimits() const;
	bool						Contains(const Angle& latitude, const Angle& longitude) const;
	bool						Contains(const LatLon& latLon) const;
	bool						ContainsRadians(double radiansLatitude, double radiansLongitude) const;
	bool						ContainsDegrees(double degreesLatitude, double degreesLongitude) const;
	bool						Contains(const Sector& that) const;

	/**
	* Determines whether this sector intersects another sector's range of latitude and longitude. The sector's angles
	* are assumed to be normalized to +/- 90 degrees latitude and +/- 180 degrees longitude. The result of the
	* operation is undefined if they are not.
	*
	* @param that the sector to test for intersection.
	*
	* @return <code>true</code> if the sectors intersect, otherwise <code>false</code>.
	*/
	bool						Intersects(const Sector& that) const;

	/**
	* Determines whether the interiors of this sector and another sector intersect. The sector's angles are assumed to
	* be normalized to +/- 90 degrees latitude and +/- 180 degrees longitude. The result of the operation is undefined
	* if they are not.
	*
	* @param that the sector to test for intersection.
	*
	* @return <code>true</code> if the sectors' interiors intersect, otherwise <code>false</code>.
	*
	* @see #intersects(Sector)
	*/
	bool						IntersectsInterior(const Sector& that) const;

	/**
	* Determines whether this sector intersects the specified geographic line segment. The line segment is specified by
	* a begin location and an end location. The locations are are assumed to be connected by a linear path in
	* geographic space. This returns true if any location along that linear path intersects this sector, including the
	* begin and end locations.
	*
	* @param begin the line segment begin location.
	* @param end   the line segment end location.
	*
	* @return true <code>true</code> if this sector intersects the line segment, otherwise <code>false</code>.
	*
	* @throws IllegalArgumentException if either the begin location or the end location is null.
	*/
	bool						IntersectsSegment(const LatLon& begin, const LatLon& end) const;
	Sector						Union(const Sector& that) const;
	Sector						Union(const Angle& latitude, const Angle& longitude) const;
	Sector						Intersection(const Sector& that) const;
	Sector						Intersection(const Angle& latitude, const Angle& longitude) const;
	void						Subdivide(Sector arr[]) const;
	std::vector<Sector>			Subdivide(int div) const;
	LatLon						Centroid() const;
	void						GetCorners(LatLon arr[]) const;

	/**
	* Creates a vector over the four corners of the sector, starting with the southwest position and continuing
	* counter-clockwise.
	*
	* @return a vector for the sector.
	*/
	std::vector<LatLon>			AsList() const;

	bool						operator==(const Sector& that) const;
	bool						operator!=(const Sector& that) const;
	
	//gmtl::Vec3d				ComputeCenterPoint(Globe globe, double exaggeration);
	//gmtl::Vec3d*				ComputeCornerPoints(Globe globe, double exaggeration);
	//static Sphere				ComputeBoundingSphere(Globe globe, double verticalExaggeration, const Sector& sector);
	//static Box				ComputeBoundingBox(Globe globe, double verticalExaggeration, Sector sector);
	//static Box				ComputeBoundingBox(Globe globe, double verticalExaggeration, Sector sector, double minElevation, double maxElevation);
	//static Cylinder			ComputeBoundingCylinder(Globe globe, double verticalExaggeration, Sector sector);
	//static Cylinder			ComputeBoundingCylinder(Globe globe, double verticalExaggeration, Sector sector, double minElevation, double maxElevation);
	//static Cylinder			ComputeBoundingCylinderOrig(Globe globe, double verticalExaggeration, Sector sector);
	//static Cylinder			ComputeBoundingCylinderOrig(Globe globe, double verticalExaggeration, Sector sector, double minElevation, double maxElevation);


private:
	Angle minLatitude;
	Angle maxLatitude;
	Angle minLongitude;
	Angle maxLongitude;
	Angle deltaLat;
	Angle deltaLon;
};

#endif