#include "Position.h"

using namespace gmtl;

Position position_zero = Position::FromDegrees(0.0, 0.0);

Position::Position() : LatLon(), elevation(0.0) { }

Position::Position(double lat, double lon, double elev) : LatLon(lat, lon), elevation(elev)
{
}

Position::Position(const Angle& lat, const Angle& lon, double elev) : LatLon(lat, lon), elevation(elev)
{
}

Position::Position(const LatLon& latLon, double elev) : LatLon(latLon), elevation(elev)
{
}

double Position::Elevation() const
{
	return elevation;
}

double Position::Altitude() const
{
	return elevation;
}

Position Position::operator+(const Position& that) const
{
	Angle lat = Angle::NormalizedLatitude(this->latitude + that.latitude);
	Angle lon = Angle::NormalizedLongitude(this->longitude + that.longitude);

	return Position(lat, lon, this->elevation + that.elevation);
}

Position Position::operator-(const Position& that) const
{
	Angle lat = Angle::NormalizedLatitude(this->latitude - that.latitude);
	Angle lon = Angle::NormalizedLongitude(this->longitude - that.longitude);

	return Position(lat, lon, this->elevation - that.elevation);
}

Position Position::Interpolate(const Position& value1, const Position& value2, double amount)
{
	if (amount < 0.0)
		return value1;
	else if (amount > 1.0)
		return value2;

	LatLon latLon = LatLon::Interpolate(value1, value2, amount);
	// Elevation is independent of geographic interpolation method (i.e. rhumb, great-circle, linear), so we
	// interpolate elevation linearly.
	double elevation;
	Math::lerp(elevation, amount, value1.Elevation(), value2.Elevation());

	return Position(latLon, elevation);
}

Position Position::InterpolateGreatCircle(const Position& value1, const Position& value2, double amount)
{
	LatLon latLon = LatLon::InterpolateGreatCircle(value1, value2, amount);
	// Elevation is independent of geographic interpolation method (i.e. rhumb, great-circle, linear), so we
	// interpolate elevation linearly.
	double elevation;
	Math::lerp(elevation, amount, value1.Elevation(), value2.Elevation());

	return Position(latLon, elevation);
}

Position Position::InterpolateRhumb(const Position& value1, const Position& value2, double amount)
{
	LatLon latLon = LatLon::InterpolateRhumb(value1, value2, amount);
	// Elevation is independent of geographic interpolation method (i.e. rhumb, great-circle, linear), so we
	// interpolate elevation linearly.
	double elevation;
	Math::lerp(elevation, amount, value1.Elevation(), value2.Elevation());

	return Position(latLon, elevation);
}