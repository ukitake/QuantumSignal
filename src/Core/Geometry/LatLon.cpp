#include "LatLon.h"
#include "..\GmtlHelpers\MathHelper.h"

using namespace gmtl;

LatLon latlon_zero = LatLon::FromDegrees(0.0, 0.0);

LatLon::LatLon(double lat, double lon) : latitude(Angle::FromDegrees(lat)), longitude(Angle::FromDegrees(lon)) { }

LatLon::LatLon() : latitude(Angle::FromDegrees(0.0)), longitude(Angle::FromDegrees(0.0)) { }

LatLon::LatLon(const Angle& lat, const Angle& lon)
{
	latitude = Angle::FromDegrees(lat.Degrees());
	longitude = Angle::FromDegrees(lon.Degrees());
}

LatLon::LatLon(const LatLon& loc)
{
	latitude = Angle::FromDegrees(loc.Latitude().Degrees());
	longitude = Angle::FromDegrees(loc.Longitude().Degrees());
}

inline Angle LatLon::Latitude() const
{
	return latitude;
}

inline Angle LatLon::Longitude() const
{
	return longitude;
}

inline LatLon LatLon::FromRadians(double latitude, double longitude)
{
	return LatLon(Math::rad2Deg(latitude), Math::rad2Deg(longitude));
}

inline LatLon LatLon::FromDegrees(double latitude, double longitude)
{
	return LatLon(latitude, longitude);
}

inline LatLon LatLon::operator+(const LatLon& other) const
{
	Angle lat = Angle::NormalizedLatitude(this->latitude + other.latitude);
	Angle lon = Angle::NormalizedLongitude(this->longitude + other.longitude);

	return LatLon(lat, lon);
}

inline LatLon LatLon::operator-(const LatLon& other) const
{
	Angle lat = Angle::NormalizedLatitude(this->latitude - other.latitude);
	Angle lon = Angle::NormalizedLongitude(this->longitude - other.longitude);

	return LatLon(lat, lon);
}

LatLon LatLon::Interpolate(const LatLon& value1, const LatLon& value2, double amount)
{
	if (value1 == value2)
		return value1;

	// todo 

	//Line line;
	//try
	//{
	//	line = Line.fromSegment(
	//		new Vec4(value1.getLongitude().radians, value1.getLatitude().radians, 0),
	//		new Vec4(value2.getLongitude().radians, value2.getLatitude().radians, 0));
	//}
	//catch (IllegalArgumentException e)
	//{
	//	// Locations became coincident after calculations.
	//	return value1;
	//}

	Vec4d p;// = line.getPointAt(amount);

	return LatLon::FromRadians(p[1], p[0]);
}

LatLon LatLon::InterpolateGreatCircle(const LatLon& value1, const LatLon& value2, double amount)
{
	if (value1 == value2)
		return value1;

	double t = Math::clamp(amount, 0.0, 1.0);
	Angle azimuth = LatLon::GreatCircleAzimuth(value1, value2);
	Angle distance = LatLon::GreatCircleDistance(value1, value2);
	Angle pathLength = Angle::FromDegrees(t * distance.Degrees());

	return LatLon::GreatCircleEndPosition(value1, azimuth, pathLength);
}

LatLon LatLon::InterpolateRhumb(const LatLon& value1, const LatLon& value2, double amount)
{
	if (value1 == value2)
		return value1;

	double t = Math::clamp(amount, 0.0, 1.0);
	Angle azimuth = LatLon::RhumbAzimuth(value1, value2);
	Angle distance = LatLon::RhumbDistance(value1, value2);
	Angle pathLength = Angle::FromDegrees(t * distance.Degrees());

	return LatLon::RhumbEndPosition(value1, azimuth, pathLength);
}

Angle LatLon::GreatCircleDistance(const LatLon& p1, const LatLon& p2)
{
	double lat1 = p1.Latitude().Radians();
	double lon1 = p1.Longitude().Radians();
	double lat2 = p2.Latitude().Radians();
	double lon2 = p2.Longitude().Radians();

	if (lat1 == lat2 && lon1 == lon2)
		return ANGLE_ZERO;

	// "Haversine formula," taken from http://en.wikipedia.org/wiki/Great-circle_distance#Formul.C3.A6
	double a = Math::sin((lat2 - lat1) / 2.0);
	double b = Math::sin((lon2 - lon1) / 2.0);
	double c = a * a + +Math::cos(lat1) * Math::cos(lat2) * b * b;
	double distanceRadians = 2.0 * Math::aSin(Math::sqrt(c));

	return MathHelper::IsNan(distanceRadians) ? Angle::FromDegrees(0.0) : Angle::FromRadians(distanceRadians);
}

Angle LatLon::GreatCircleAzimuth(const LatLon& p1, const LatLon& p2)
{
	double lat1 = p1.Latitude().Radians();
	double lon1 = p1.Longitude().Radians();
	double lat2 = p2.Latitude().Radians();
	double lon2 = p2.Longitude().Radians();

	if (lat1 == lat2 && lon1 == lon2)
		return ANGLE_ZERO;

	if (lon1 == lon2)
		return lat1 > lat2 ? Angle::FromDegrees(180.0) : ANGLE_ZERO;

	// Taken from "Map Projections - A Working Manual", page 30, equation 5-4b.
	// The atan2() function is used in place of the traditional atan(y/x) to simplify the case when x==0.
	double y = Math::cos(lat2) * Math::sin(lon2 - lon1);
	double x = Math::cos(lat1) * Math::sin(lat2) - Math::sin(lat1) * Math::cos(lat2) * Math::cos(lon2 - lon1);
	double azimuthRadians = Math::aTan2(y, x);

	return MathHelper::IsNan(azimuthRadians) ? Angle::FromDegrees(0.0) : Angle::FromRadians(azimuthRadians);
}

LatLon LatLon::GreatCircleEndPosition(const LatLon& p, const Angle& greatCircleAzimuth, const Angle& pathLength)
{
	double lat = p.Latitude().Radians();
	double lon = p.Longitude().Radians();
	double azimuth = greatCircleAzimuth.Radians();
	double distance = pathLength.Radians();

	if (distance == 0.0)
		return p;

	// Taken from "Map Projections - A Working Manual", page 31, equation 5-5 and 5-6.
	double endLatRadians = Math::aSin(Math::sin(lat) * Math::cos(distance)
		+ Math::cos(lat) * Math::sin(distance) * Math::cos(azimuth));
	double endLonRadians = lon + Math::aTan2(
		Math::sin(distance) * Math::sin(azimuth),
		Math::cos(lat) * Math::cos(distance) - Math::sin(lat) * Math::sin(distance) * Math::cos(azimuth));

	if (MathHelper::IsNan(endLatRadians) || MathHelper::IsNan(endLonRadians))
		return p;

	return LatLon(
		Angle::FromRadians(endLatRadians).NormalizedLatitude(),
		Angle::FromRadians(endLonRadians).NormalizedLongitude());
}

LatLon LatLon::GreatCircleEndPosition(const LatLon& p, double greatCircleAzimuthRadians, double pathLengthRadians)
{
	return GreatCircleEndPosition(p,
		Angle::FromRadians(greatCircleAzimuthRadians), Angle::FromRadians(pathLengthRadians));
}

LatLon* LatLon::GreatCircleExtremeLocations(const LatLon& location, const Angle& azimuth, LatLon arr[])
{
	double lat0 = location.Latitude().Radians();
	double az = azimuth.Radians();

	// Derived by solving the function for longitude on a great circle against the desired longitude. We start with
	// the equation in "Map Projections - A Working Manual", page 31, equation 5-5:
	//
	// lat = asin( sin(lat0) * cos(c) + cos(lat0) * sin(c) * cos(Az) )
	//
	// Where (lat0, lon) are the starting coordinates, c is the angular distance along the great circle from the
	// starting coordinate, and Az is the azimuth. All values are in radians.
	//
	// Solving for angular distance gives distance to the equator:
	//
	// tan(c) = -tan(lat0) / cos(Az)
	//
	// The great circle is by definition centered about the Globe's origin. Therefore intersections with the
	// equator will be antipodal (exactly 180 degrees opposite each other), as will be the extreme latitudes.
	// My observing the symmetry of a great circle, it is also apparent that the extreme latitudes will be 90
	// degrees from either interseciton with the equator.
	//
	// d1 = c + 90
	// d2 = c - 90

	double tanDistance = -Math::tan(lat0) / Math::cos(az);
	double distance = Math::aTan(tanDistance);

	Angle extremeDistance1 = Angle::FromRadians(distance + (Math::PI / 2.0));
	Angle extremeDistance2 = Angle::FromRadians(distance - (Math::PI / 2.0));

	arr[0] = GreatCircleEndPosition(location, azimuth, extremeDistance1);
	arr[1] = GreatCircleEndPosition(location, azimuth, extremeDistance2);
	return arr;
}

LatLon* LatLon::GreatCircleExtremeLocations(const LatLon& begin, const LatLon& end, LatLon arr[])
{
	LatLon minLatLocation;
	LatLon maxLatLocation;
	double minLat = 90.0;
	double maxLat = -90.0;

	// Compute the min and max latitude and assocated locations from the arc endpoints.
	if (minLat >= begin.Latitude().Degrees())
	{
		minLat = begin.Latitude().Degrees();
		minLatLocation = begin;
	}
	if (maxLat <= begin.Latitude().Degrees())
	{
		maxLat = begin.Latitude().Degrees();
		maxLatLocation = begin;
	}
	if (minLat >= end.Latitude().Degrees())
	{
		minLat = end.Latitude().Degrees();
		minLatLocation = end;
	}
	if (maxLat <= end.Latitude().Degrees())
	{
		maxLat = end.Latitude().Degrees();
		maxLatLocation = end;
	}

	// Compute parameters for the great circle arc defined by begin and end. Then compute the locations of extreme
	// latitude on entire the great circle which that arc is part of.
	Angle greatArcAzimuth = GreatCircleAzimuth(begin, end);
	Angle greatArcDistance = GreatCircleDistance(begin, end);

	LatLon greatCircleExtremes[2];
	GreatCircleExtremeLocations(begin, greatArcAzimuth, greatCircleExtremes);

	// Determine whether either of the extreme locations are inside the arc defined by begin and end. If so,
	// adjust the min and max latitude accordingly.
	for (LatLon ll : greatCircleExtremes)
	{
		Angle az = LatLon::GreatCircleAzimuth(begin, ll);
		Angle d = LatLon::GreatCircleDistance(begin, ll);

		// The extreme location must be between the begin and end locations. Therefore its azimuth relative to
		// the begin location should have the same signum, and its distance relative to the begin location should
		// be between 0 and greatArcDistance, inclusive.
		if (Math::sign(az.Degrees()) == Math::sign(greatArcAzimuth.Degrees()))
		{
			if (d.Degrees() >= 0.0 && d.Degrees() <= greatArcDistance.Degrees())
			{
				if (minLat >= ll.Latitude().Degrees())
				{
					minLat = ll.Latitude().Degrees();
					minLatLocation = ll;
				}
				if (maxLat <= ll.Latitude().Degrees())
				{
					maxLat = ll.Latitude().Degrees();
					maxLatLocation = ll;
				}
			}
		}
	}

	arr[0] = minLatLocation;
	arr[1] = maxLatLocation;
	return arr;
}

Angle LatLon::RhumbDistance(const LatLon& p1, const LatLon& p2)
{
	double lat1 = p1.Latitude().Radians();
	double lon1 = p1.Longitude().Radians();
	double lat2 = p2.Latitude().Radians();
	double lon2 = p2.Longitude().Radians();

	if (lat1 == lat2 && lon1 == lon2)
		return ANGLE_ZERO;

	// Taken from http://www.movable-type.co.uk/scripts/latlong.html
	double dLat = lat2 - lat1;
	double dLon = lon2 - lon1;
	double dPhi = Math::log(Math::tan(lat2 / 2.0 + Math::PI / 4.0) / Math::tan(lat1 / 2.0 + Math::PI / 4.0));
	double q = dLat / dPhi;
	if (MathHelper::IsNan(dPhi) || MathHelper::IsNan(q))
	{
		q = Math::cos(lat1);
	}
	// If lonChange over 180 take shorter rhumb across 180 meridian.
	if (Math::abs(dLon) > Math::PI)
	{
		dLon = dLon > 0.0 ? -(2.0 * Math::PI - dLon) : (2.0 * Math::PI + dLon);
	}

	double distanceRadians = Math::sqrt(dLat * dLat + q * q * dLon * dLon);

	return MathHelper::IsNan(distanceRadians) ? Angle::FromDegrees(0.0) : Angle::FromRadians(distanceRadians);
}

Angle LatLon::RhumbAzimuth(const LatLon& p1, const LatLon& p2)
{
	double lat1 = p1.Latitude().Radians();
	double lon1 = p1.Longitude().Radians();
	double lat2 = p2.Latitude().Radians();
	double lon2 = p2.Longitude().Radians();

	if (lat1 == lat2 && lon1 == lon2)
		return ANGLE_ZERO;

	// Taken from http://www.movable-type.co.uk/scripts/latlong.html
	double dLon = lon2 - lon1;
	double dPhi = Math::log(Math::tan(lat2 / 2.0 + Math::PI / 4.0) / Math::tan(lat1 / 2.0 + Math::PI / 4.0));
	// If lonChange over 180 take shorter rhumb across 180 meridian.
	if (Math::abs(dLon) > Math::PI)
	{
		dLon = dLon > 0.0 ? -(2.0 * Math::PI - dLon) : (2.0 * Math::PI + dLon);
	}
	double azimuthRadians = Math::aTan2(dLon, dPhi);

	return MathHelper::IsNan(azimuthRadians) ? Angle::FromDegrees(0.0) : Angle::FromRadians(azimuthRadians);
}

LatLon LatLon::RhumbEndPosition(const LatLon& p, const Angle& rhumbAzimuth, const Angle& pathLength)
{
	double lat1 = p.Latitude().Radians();
	double lon1 = p.Longitude().Radians();
	double azimuth = rhumbAzimuth.Radians();
	double distance = pathLength.Radians();

	if (distance == 0)
		return p;

	// Taken from http://www.movable-type.co.uk/scripts/latlong.html
	double lat2 = lat1 + distance * Math::cos(azimuth);
	double dPhi = Math::log(Math::tan(lat2 / 2.0 + Math::PI / 4.0) / Math::tan(lat1 / 2.0 + Math::PI / 4.0));
	double q = (lat2 - lat1) / dPhi;
	if (MathHelper::IsNan(dPhi) || MathHelper::IsNan(q) || MathHelper::IsInf(q))
	{
		q = Math::cos(lat1);
	}
	double dLon = distance * Math::sin(azimuth) / q;
	// Handle latitude passing over either pole.
	if (Math::abs(lat2) > Math::PI / 2.0)
	{
		lat2 = lat2 > 0 ? Math::PI - lat2 : -Math::PI - lat2;
	}
	double lon2 = fmod((lon1 + dLon + Math::PI), (2.0 * Math::PI)) - Math::PI;

	if (MathHelper::IsNan(lat2) || MathHelper::IsNan(lon2))
		return p;

	return LatLon(
		Angle::FromRadians(lat2).NormalizedLatitude(),
		Angle::FromRadians(lon2).NormalizedLongitude());
}

LatLon LatLon::RhumbEndPosition(const LatLon& p, double rhumbAzimuthRadians, double pathLengthRadians)
{
	return RhumbEndPosition(p, Angle::FromRadians(rhumbAzimuthRadians), Angle::FromRadians(pathLengthRadians));
}

bool LatLon::operator==(const LatLon& other) const
{
	return this->latitude == other.latitude && this->longitude == other.longitude;
}

bool LatLon::operator!=(const LatLon& other) const
{
	return this->latitude != other.latitude || this->longitude != other.longitude;
}

bool LatLon::LocationsCrossDateline(const LatLon& p1, const LatLon& p2)
{
	// A segment cross the line if end pos have different longitude signs
	// and are more than 180 degrees longitude apart
	if (Math::sign(p1.Longitude().Degrees()) != Math::sign(p2.Longitude().Degrees()))
	{
		double delta = Math::abs(p1.Longitude().Degrees() - p2.Longitude().Degrees());
		if (delta > 180.0 && delta < 360.0)
			return true;
	}

	return false;
}

Angle LatLon::EllipsoidalForwardAzimuth(const LatLon& p1, const LatLon& p2, double equatorialRadius, double polarRadius)
{
	// TODO: What if polar radius is larger than equatorial radius?
	// Calculate flattening
	double f = (equatorialRadius - polarRadius) / equatorialRadius; // flattening

	// Calculate reduced latitudes and related sines/cosines
	double U1 = Math::aTan((1.0 - f) * Math::tan(p1.latitude.Radians()));
	double cU1 = Math::cos(U1);
	double sU1 = Math::sin(U1);

	double U2 = Math::aTan((1.0 - f) * Math::tan(p2.latitude.Radians()));
	double cU2 = Math::cos(U2);
	double sU2 = Math::sin(U2);

	// Calculate difference in longitude
	double L = (p2.longitude - p1.longitude).Radians();

	// Vincenty's Formula for Forward Azimuth
	// iterate until change in lambda is negligible (e.g. 1e-12 ~= 0.06mm)
	// first approximation
	double lambda = L;
	double sLambda = Math::sin(lambda);
	double cLambda = Math::cos(lambda);

	// dummy value to ensure
	double lambda_prev = DBL_MAX;
	int count = 0;
	while (Math::abs(lambda - lambda_prev) > 1e-12 && count++ < 100)
	{
		// Store old lambda
		lambda_prev = lambda;
		// Calculate new lambda
		double sSigma = Math::sqrt(Math::pow(cU2 * sLambda, 2)
			+ Math::pow(cU1 * sU2 - sU1 * cU2 * cLambda, 2));
		double cSigma = sU1 * sU2 + cU1 * cU2 * cLambda;
		double sigma = Math::aTan2(sSigma, cSigma);
		double sAlpha = cU1 * cU2 * sLambda / sSigma;
		double cAlpha2 = 1 - sAlpha * sAlpha; // trig identity
		// As cAlpha2 approaches zeros, set cSigmam2 to zero to converge on a solution
		double cSigmam2;
		if (Math::abs(cAlpha2) < 1e-6)
		{
			cSigmam2 = 0.0;
		}
		else
		{
			cSigmam2 = cSigma - 2.0 * sU1 * sU2 / cAlpha2;
		}
		double c = f / 16.0 * cAlpha2 * (4.0 + f * (4.0 - 3.0 * cAlpha2));

		lambda = L + (1.0 - c) * f * sAlpha * (sigma + c * sSigma * (cSigmam2 + c * cSigma * (-1 + 2 * cSigmam2)));
		sLambda = Math::sin(lambda);
		cLambda = Math::cos(lambda);
	}

	return Angle::FromRadians(Math::aTan2(cU2 * sLambda, cU1 * sU2 - sU1 * cU2 * cLambda));
}

double LatLon::EllipsoidalDistance(const LatLon& p1, const LatLon& p2, double equatorialRadius, double polarRadius)
{
	// TODO: I think there is a non-iterative way to calculate the distance. Find it and compare with this one.
	// TODO: What if polar radius is larger than equatorial radius?
	double F = (equatorialRadius - polarRadius) / equatorialRadius; // flattening = 1.0 / 298.257223563;
	double R = 1.0 - F;
	double EPS = 0.5E-13;

	// Algorithm from National Geodetic Survey, FORTRAN program "inverse,"
	// subroutine "INVER1," by L. PFEIFER and JOHN G. GERGEN.
	// http://www.ngs.noaa.gov/TOOLS/Inv_Fwd/Inv_Fwd.html
	// Conversion to JAVA from FORTRAN was made with as few changes as possible
	// to avoid errors made while recasting form, and to facilitate any future
	// comparisons between the original code and the altered version in Java.
	// Original documentation:
	// SOLUTION OF THE GEODETIC INVERSE PROBLEM AFTER T.VINCENTY
	// MODIFIED RAINSFORD'S METHOD WITH HELMERT'S ELLIPTICAL TERMS
	// EFFECTIVE IN ANY AZIMUTH AND AT ANY DISTANCE SHORT OF ANTIPODAL
	// STANDPOINT/FOREPOINT MUST NOT BE THE GEOGRAPHIC POLE
	// A IS THE SEMI-MAJOR AXIS OF THE REFERENCE ELLIPSOID
	// F IS THE FLATTENING (NOT RECIPROCAL) OF THE REFERNECE ELLIPSOID
	// LATITUDES GLAT1 AND GLAT2
	// AND LONGITUDES GLON1 AND GLON2 ARE IN RADIANS POSITIVE NORTH AND EAST
	// FORWARD AZIMUTHS AT BOTH POINTS RETURNED IN RADIANS FROM NORTH
	//
	// Reference ellipsoid is the WGS-84 ellipsoid.
	// See http://www.colorado.edu/geography/gcraft/notes/datum/elist.html
	// FAZ is forward azimuth in radians from pt1 to pt2;
	// BAZ is backward azimuth from point 2 to 1;
	// S is distance in meters.
	//
	// Conversion to JAVA from FORTRAN was made with as few changes as possible
	// to avoid errors made while recasting form, and to facilitate any future
	// comparisons between the original code and the altered version in Java.
	//
	//IMPLICIT REAL*8 (A-H,O-Z)
	//  COMMON/CONST/PI,RAD

	double GLAT1 = p1.Latitude().Radians();
	double GLAT2 = p2.Latitude().Radians();
	double TU1 = R * Math::sin(GLAT1) / Math::cos(GLAT1);
	double TU2 = R * Math::sin(GLAT2) / Math::cos(GLAT2);
	double CU1 = 1. / Math::sqrt(TU1 * TU1 + 1.);
	double SU1 = CU1 * TU1;
	double CU2 = 1. / Math::sqrt(TU2 * TU2 + 1.);
	double S = CU1 * CU2;
	double BAZ = S * TU2;
	double FAZ = BAZ * TU1;
	double GLON1 = p1.Longitude().Radians();
	double GLON2 = p2.Longitude().Radians();
	double X = GLON2 - GLON1;
	double D, SX, CX, SY, CY, Y, SA, C2A, CZ, E, C;
	do
	{
		SX = Math::sin(X);
		CX = Math::cos(X);
		TU1 = CU2 * SX;
		TU2 = BAZ - SU1 * CU2 * CX;
		SY = Math::sqrt(TU1 * TU1 + TU2 * TU2);
		CY = S * CX + FAZ;
		Y = Math::aTan2(SY, CY);
		SA = S * SX / SY;
		C2A = -SA * SA + 1.;
		CZ = FAZ + FAZ;
		if (C2A > 0.)
		{
			CZ = -CZ / C2A + CY;
		}
		E = CZ * CZ * 2. - 1.;
		C = ((-3. * C2A + 4.) * F + 4.) * C2A * F / 16.;
		D = X;
		X = ((E * CY * C + CZ) * SY * C + Y) * SA;
		X = (1. - C) * X * F + GLON2 - GLON1;
		//IF(DABS(D-X).GT.EPS) GO TO 100
	} while (Math::abs(D - X) > EPS);

	//FAZ = Math::atan2(TU1, TU2);
	//BAZ = Math::atan2(CU1 * SX, BAZ * CX - SU1 * CU2) + Math::PI;
	X = Math::sqrt((1. / R / R - 1.) * C2A + 1.) + 1.;
	X = (X - 2.) / X;
	C = 1. - X;
	C = (X * X / 4. + 1.) / C;
	D = (0.375 * X * X - 1.) * X;
	X = E * CY;
	S = 1. - E - E;
	S = ((((SY * SY * 4. - 3.) * S * CZ * D / 6. - X) * D / 4. + CZ) * SY
		* D + Y) * C * equatorialRadius * R;

	return S;
}