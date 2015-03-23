#ifndef CORE_GEOMETRY_LATLON_H
#define CORE_GEOMETRY_LATLON_H

#include "..\Core.h"
#include "Angle.h"

class Angle;

class LatLon
{
protected:
	Angle					latitude;
	Angle					longitude;

	LatLon(double lat, double lon);

public:

	static LatLon			FromRadians(double latitude, double longitude);
	static LatLon			FromDegrees(double latitude, double longitude);

	LatLon();
	LatLon(const Angle& lat, const Angle& lon);
	LatLon(const LatLon& l);

	/**
	* Returns the linear interpolation of <code>value1</code> and <code>value2</code>, treating the geographic
	* locations as simple 2D coordinate pairs.
	*
	* @param amount the interpolation factor
	* @param value1 the first location.
	* @param value2 the second location.
	*
	* @return the linear interpolation of <code>value1</code> and <code>value2</code>.
	*
	* @throws IllegalArgumentException if either location is null.
	*/
	static LatLon			Interpolate(const LatLon& l1, const LatLon& l2, double amount);

	/**
	* Returns the an interpolated location along the great-arc between <code>value1</code> and <code>value2</code>. The
	* interpolation factor <code>amount</code> defines the weight given to each value, and is clamped to the range [0,
	* 1]. If <code>a</code> is 0 or less, this returns <code>value1</code>. If <code>amount</code> is 1 or more, this
	* returns <code>value2</code>. Otherwise, this returns the location on the great-arc between <code>value1</code>
	* and <code>value2</code> corresponding to the specified interpolation factor.
	*
	* @param amount the interpolation factor
	* @param value1 the first location.
	* @param value2 the second location.
	*
	* @return an interpolated location along the great-arc between <code>value1</code> and <code>value2</code>.
	*
	* @throws IllegalArgumentException if either location is null.
	*/
	static LatLon			InterpolateGreatCircle(const LatLon& l1, const LatLon& l2, double amount);

	/**
	* Returns the an interpolated location along the rhumb line between <code>value1</code> and <code>value2</code>.
	* The interpolation factor <code>amount</code> defines the weight given to each value, and is clamped to the range
	* [0, 1]. If <code>a</code> is 0 or less, this returns <code>value1</code>. If <code>amount</code> is 1 or more,
	* this returns <code>value2</code>. Otherwise, this returns the location on the rhumb line between
	* <code>value1</code> and <code>value2</code> corresponding to the specified interpolation factor.
	*
	* @param amount the interpolation factor
	* @param value1 the first location.
	* @param value2 the second location.
	*
	* @return an interpolated location along the rhumb line between <code>value1</code> and <code>value2</code>
	*
	* @throws IllegalArgumentException if either location is null.
	*/
	static LatLon			InterpolateRhumb(const LatLon& l1, const LatLon& l2, double amount);

	/**
	* Computes the great circle angular distance between two locations. The return value gives the distance as the
	* angle between the two positions on the pi radius circle. In radians, this angle is also the arc length of the
	* segment between the two positions on that circle. To compute a distance in meters from this value, multiply it by
	* the radius of the globe.
	*
	* @param p1 LatLon of the first location
	* @param p2 LatLon of the second location
	*
	* @return the angular distance between the two locations. In radians, this value is the arc length on the radius pi
	*         circle.
	*/
	static Angle			GreatCircleDistance(const LatLon& start, const LatLon& dest);

	/**
	* Computes the azimuth angle (clockwise from North) that points from the first location to the second location.
	* This angle can be used as the starting azimuth for a great circle arc that begins at the first location, and
	* passes through the second location.
	*
	* @param p1 LatLon of the first location
	* @param p2 LatLon of the second location
	*
	* @return Angle that points from the first location to the second location.
	*/
	static Angle			GreatCircleAzimuth(const LatLon& start, const LatLon& dest);

	/**
	* Computes the location on a great circle arc with the given starting location, azimuth, and arc distance.
	*
	* @param p                  LatLon of the starting location
	* @param greatCircleAzimuth great circle azimuth angle (clockwise from North)
	* @param pathLength         arc distance to travel
	*
	* @return LatLon location on the great circle arc.
	*/
	static LatLon			GreatCircleEndPosition(const LatLon& p, const Angle& greatCircleAzimuth, const Angle& pathLength);

	/**
	* Computes the location on a great circle arc with the given starting location, azimuth, and arc distance.
	*
	* @param p                         LatLon of the starting location
	* @param greatCircleAzimuthRadians great circle azimuth angle (clockwise from North), in radians
	* @param pathLengthRadians         arc distance to travel, in radians
	*
	* @return LatLon location on the great circle arc.
	*/
	static LatLon			GreatCircleEndPosition(const LatLon& p, double greatCircleAzimuthRadians, double pathLengthRadians);

	/**
	* Returns two locations with the most extreme latitudes on the great circle with the given starting location and
	* azimuth.
	*
	* @param location location on the great circle.
	* @param azimuth  great circle azimuth angle (clockwise from North).
	*
	* @return two locations where the great circle has its extreme latitudes.
	*
	* @throws IllegalArgumentException if either <code>location</code> or <code>azimuth</code> are null.
	*/
	static LatLon*			GreatCircleExtremeLocations(const LatLon& location, const Angle& azimuth, LatLon arr[]);

	/**
	* Returns two locations with the most extreme latitudes on the great circle arc defined by, and limited to, the two
	* locations.
	*
	* @param begin beginning location on the great circle arc.
	* @param end   ending location on the great circle arc.
	*
	* @return two locations with the most extreme latitudes on the great circle arc.
	*
	* @throws IllegalArgumentException if either <code>begin</code> or <code>end</code> are null.
	*/
	static LatLon*			GreatCircleExtremeLocations(const LatLon& begin, const LatLon& end, LatLon arr[]);
	

	/**
	* Computes the length of the rhumb line between two locations. The return value gives the distance as the angular
	* distance between the two positions on the pi radius circle. In radians, this angle is also the arc length of the
	* segment between the two positions on that circle. To compute a distance in meters from this value, multiply it by
	* the radius of the globe.
	*
	* @param p1 LatLon of the first location
	* @param p2 LatLon of the second location
	*
	* @return the arc length of the rhumb line between the two locations. In radians, this value is the arc length on
	*         the radius pi circle.
	*/
	static Angle			RhumbDistance(const LatLon& start, const LatLon& dest);

	/**
	* Computes the azimuth angle (clockwise from North) of a rhumb line (a line of constant heading) between two
	* locations.
	*
	* @param p1 LatLon of the first location
	* @param p2 LatLon of the second location
	*
	* @return azimuth Angle of a rhumb line between the two locations.
	*/
	static Angle			RhumbAzimuth(const LatLon& start, const LatLon& dest);

	/**
	* Computes the location on a rhumb line with the given starting location, rhumb azimuth, and arc distance along the
	* line.
	*
	* @param p            LatLon of the starting location
	* @param rhumbAzimuth rhumb azimuth angle (clockwise from North)
	* @param pathLength   arc distance to travel
	*
	* @return LatLon location on the rhumb line.
	*/
	static LatLon			RhumbEndPosition(const LatLon& p, const Angle& rhumbAzimuth, const Angle& pathLength);

	/**
	* Computes the location on a rhumb line with the given starting location, rhumb azimuth, and arc distance along the
	* line.
	*
	* @param p                   LatLon of the starting location
	* @param rhumbAzimuthRadians rhumb azimuth angle (clockwise from North), in radians
	* @param pathLengthRadians   arc distance to travel, in radians
	*
	* @return LatLon location on the rhumb line.
	*/
	static LatLon			RhumbEndPosition(const LatLon& p, double rhumbAzimuthRadians, double pathLengthRadians);


	/**
	* Compute the forward azimuth between two positions
	*
	* @param p1               first position
	* @param p2               second position
	* @param equatorialRadius the equatorial radius of the globe in meters
	* @param polarRadius      the polar radius of the globe in meters
	*
	* @return the azimuth
	*/
	static Angle			EllipsoidalForwardAzimuth(const LatLon& p1, const LatLon& p2, double equatorialRadius, double polarRadius);

	/**
	* Computes the distance between two points on an ellipsoid iteratively.
	* <p/>
	* NOTE: This method was copied from the UniData NetCDF Java library. http://www.unidata.ucar.edu/software/netcdf-java/
	* <p/>
	* Algorithm from U.S. National Geodetic Survey, FORTRAN program "inverse," subroutine "INVER1," by L. PFEIFER and
	* JOHN G. GERGEN. See http://www.ngs.noaa.gov/TOOLS/Inv_Fwd/Inv_Fwd.html
	* <p/>
	* Original documentation: SOLUTION OF THE GEODETIC INVERSE PROBLEM AFTER T.VINCENTY MODIFIED RAINSFORD'S METHOD
	* WITH HELMERT'S ELLIPTICAL TERMS EFFECTIVE IN ANY AZIMUTH AND AT ANY DISTANCE SHORT OF ANTIPODAL
	* STANDPOINT/FOREPOINT MUST NOT BE THE GEOGRAPHIC POLE
	* <p/>
	* Requires close to 1.4 E-5 seconds wall clock time per call on a 550 MHz Pentium with Linux 7.2.
	*
	* @param p1               first position
	* @param p2               second position
	* @param equatorialRadius the equatorial radius of the globe in meters
	* @param polarRadius      the polar radius of the globe in meters
	*
	* @return distance in meters between the two points
	*/
	static double			EllipsoidalDistance(const LatLon& p1, const LatLon& p2, double equatorialRadius, double polarRadius);

	static bool				LocationsCrossDateline(const LatLon& start, const LatLon& dest);

	LatLon					operator+(const LatLon& other) const;
	LatLon					operator-(const LatLon& other) const;

	bool					operator==(const LatLon& other) const;
	bool					operator!=(const LatLon& other) const;

	Angle					Latitude() const;
	Angle					Longitude() const;
};

extern LatLon latlon_zero;
#define LATLON_ZERO latlon_zero;

#endif /* CORE_GEOMETRY_LATLON_H */