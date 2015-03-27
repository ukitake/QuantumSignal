#ifndef CORE_GEOMETRY_POSITION_H
#define CORE_GEOMETRY_POSITION_H

#include "..\Core.h"
#include "LatLon.h"
#include "..\GmtlHelpers\MatrixHelper.h"

class Position : public LatLon
{
protected:
	double					elevation;

	Position(double lat, double lon, double elev);

public:
	static Position			FromRadians(double latitude, double longitude, double elevation);
	static Position			FromDegrees(double latitude, double longitude, double elevation);
	static Position			FromDegrees(double latitude, double longitude);
	
	/**
	* Returns the linear interpolation of <code>value1</code> and <code>value2</code>, treating the geographic
	* locations as simple 2D coordinate pairs, and treating the elevation values as 1D scalars.
	*
	* @param amount the interpolation factor
	* @param value1 the first position.
	* @param value2 the second position.
	*
	* @return the linear interpolation of <code>value1</code> and <code>value2</code>.
	*
	* @throws IllegalArgumentException if either position is null.
	*/
	static Position			Interpolate(const Position& value1, const Position& value2, double amount);

	/**
	* Returns the an interpolated location along the great-arc between <code>value1</code> and <code>value2</code>. The
	* position's elevation components are linearly interpolated as a simple 1D scalar value. The interpolation factor
	* <code>amount</code> defines the weight given to each value, and is clamped to the range [0, 1]. If <code>a</code>
	* is 0 or less, this returns <code>value1</code>. If <code>amount</code> is 1 or more, this returns
	* <code>value2</code>. Otherwise, this returns the position on the great-arc between <code>value1</code> and
	* <code>value2</code> with a linearly interpolated elevation component, and corresponding to the specified
	* interpolation factor.
	*
	* @param amount the interpolation factor
	* @param value1 the first position.
	* @param value2 the second position.
	*
	* @return an interpolated position along the great-arc between <code>value1</code> and <code>value2</code>, with a
	*         linearly interpolated elevation component.
	*
	* @throws IllegalArgumentException if either location is null.
	*/
	static Position			InterpolateGreatCircle(const Position& value1, const Position& value2, double amount);

	/**	
	* Returns the an interpolated location along the rhumb line between <code>value1</code> and <code>value2</code>.
	* The position's elevation components are linearly interpolated as a simple 1D scalar value. The interpolation
	* factor <code>amount</code> defines the weight given to each value, and is clamped to the range [0, 1]. If
	* <code>a</code> is 0 or less, this returns <code>value1</code>. If <code>amount</code> is 1 or more, this returns
	* <code>value2</code>. Otherwise, this returns the position on the rhumb line between <code>value1</code> and
	* <code>value2</code> with a linearly interpolated elevation component, and corresponding to the specified
	* interpolation factor.
	*
	* @param amount the interpolation factor
	* @param value1 the first position.
	* @param value2 the second position.
	*
	* @return an interpolated position along the great-arc between <code>value1</code> and <code>value2</code>, with a
	*         linearly interpolated elevation component.
	*
	* @throws IllegalArgumentException if either location is null.
	*/
	static Position			InterpolateRhumb(const Position& value1, const Position& value2, double amount);

	Position();
	Position(const Angle& lat, const Angle& lon, double elev);
	Position(const LatLon& latLon, double elev);

	double					Elevation() const;
	double					Altitude() const;

	Position				operator+(const Position& that) const;
	Position				operator-(const Position& that) const;

};

extern Position position_zero;
#define POSITION_ZERO position_zero

#endif