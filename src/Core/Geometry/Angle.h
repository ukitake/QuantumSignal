#ifndef CORE_GEOMETRY_ANGLE_H
#define CORE_GEOMETRY_ANGLE_H

#include "..\Core.h"

class Angle {
private:
	double				degrees;
	double				radians;

	Angle(double degrees, double radians) : degrees(degrees), radians(radians) { }

	static double		NormalizedDegreesLatitude(double degrees);
	static double		NormalizedDegreesLongitude(double degrees);

public:

	Angle() : degrees(0.0), radians(0.0) { }
	Angle(const Angle& angle) : degrees(angle.degrees), radians(angle.radians) { }

	static Angle		FromDegrees(double degrees);
	static Angle		FromRadians(double radians);
	static Angle		MidAngle(const Angle& a1, const Angle& a2);	// average of 2 Angles
	static Angle		Lerp(const Angle& a1, const Angle& a2, double factor);
	
	static Angle		NormalizedLatitude(const Angle& unnormalizedAngle);
	static Angle		NormalizedLongitude(const Angle& unnormalizedAngle);

	inline static bool	IsValidLatitude(double value) { return value >= -90.0 && value <= 90.0; }
	inline static bool	IsValidLongitude(double value) { return value >= -180.0 && value <= 180.0; }
	inline static Angle	Max(const Angle& a, const Angle& b) { return a.degrees >= b.degrees ? a : b; }
	inline static Angle Min(const Angle& a, const Angle& b) { return a.degrees <= b.degrees ? a : b; }

	inline double		Degrees() const { return degrees; }
	inline double		Radians() const { return radians; }
	inline double		Sin() const { return gmtl::Math::sin(radians); }
	inline double		SinHalfAngle() const { return gmtl::Math::sin(0.5 * radians); }
	inline double		Cos() const { return gmtl::Math::cos(radians); }
	inline double		CosHalfAngle() const { return gmtl::Math::cos(0.5 * radians); }
	inline double		TanHalfAngle() const { return gmtl::Math::tan(0.5 * radians); }

	Angle				operator-() const;
	Angle				operator+(const Angle& angle) const;
	Angle				operator-(const Angle& angle) const;
	Angle				operator*(const Angle& angle) const;
	Angle				operator*(double multFactor) const;
	Angle				operator/(const Angle& angle) const;
	Angle				operator/(double divFactor) const;

	bool				operator==(const Angle& other) const;
	bool				operator!=(const Angle& other) const;
	
	Angle				AngularDistanceTo(const Angle& dest) const;
	Angle				NormalizedLatitude() const;
	Angle				NormalizedLongitude() const;

};

extern Angle angle_zero;
#define ANGLE_ZERO angle_zero;

#endif /* CORE_GEOMETRY_ANGLE_H */