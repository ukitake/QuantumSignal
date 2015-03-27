#include "Sector.h"

using namespace gmtl;

Sector::Sector()
{
	minLatitude = Angle();
	minLongitude = Angle();
	maxLatitude = Angle();
	maxLongitude = Angle();
	deltaLat = Angle();
	deltaLon = Angle();
}

Sector::Sector(const Angle& minLatitude, const Angle& maxLatitude, const Angle& minLongitude, const Angle& maxLongitude)
{
	this->minLatitude = Angle(minLatitude);
	this->minLongitude = Angle(minLongitude);
	this->maxLatitude = Angle(maxLatitude);
	this->maxLongitude = Angle(maxLongitude);
	this->deltaLat = Angle::FromDegrees(this->maxLatitude.Degrees() - this->minLatitude.Degrees());
	this->deltaLon = Angle::FromDegrees(this->maxLongitude.Degrees() - this->minLongitude.Degrees());
}

Sector::Sector(const Sector& sector)
{
	minLatitude = Angle(sector.minLatitude);
	minLongitude = Angle(sector.minLongitude);
	maxLatitude = Angle(sector.maxLatitude);
	maxLongitude = Angle(sector.maxLongitude);
	deltaLat = Angle(sector.deltaLat);
	deltaLon = Angle(sector.deltaLon);
}

Sector Sector::FromDegrees(double minLatitude, double maxLatitude, double minLongitude, double maxLongitude)
{
	return Sector(Angle::FromDegrees(minLatitude), Angle::FromDegrees(maxLatitude), Angle::FromDegrees(minLongitude), Angle::FromDegrees(maxLongitude));
}

Sector Sector::FromRadians(double minLatitude, double maxLatitude, double minLongitude, double maxLongitude)
{
	return Sector(Angle::FromRadians(minLatitude), Angle::FromRadians(maxLatitude), Angle::FromRadians(minLongitude), Angle::FromRadians(maxLongitude));
}

Sector Sector::BoundingSector(const std::vector<LatLon>& locations)
{
	if (locations.size() == 0)
		return Sector(); 

	double minLat = 90.0;
	double minLon = 180.0;
	double maxLat = -90.0;
	double maxLon = -180.0;

	for (LatLon p : locations)
	{
		double lat = p.Latitude().Degrees();
		if (lat < minLat)
			minLat = lat;
		if (lat > maxLat)
			maxLat = lat;

		double lon = p.Longitude().Degrees();
		if (lon < minLon)
			minLon = lon;
		if (lon > maxLon)
			maxLon = lon;
	}

	return Sector::FromDegrees(minLat, maxLat, minLon, maxLon);
}

Sector Sector::BoundingSector(const LatLon& pA, const LatLon& pB)
{
	double minLat = pA.Latitude().Degrees();
	double minLon = pA.Longitude().Degrees();
	double maxLat = pA.Latitude().Degrees();
	double maxLon = pA.Longitude().Degrees();

	if (pB.Latitude().Degrees() < minLat)
		minLat = pB.Latitude().Degrees();
	else if (pB.Latitude().Degrees() > maxLat)
		maxLat = pB.Latitude().Degrees();

	if (pB.Longitude().Degrees() < minLon)
		minLon = pB.Longitude().Degrees();
	else if (pB.Longitude().Degrees() > maxLon)
		maxLon = pB.Longitude().Degrees();

	return Sector::FromDegrees(minLat, maxLat, minLon, maxLon);
}

std::vector<Sector>	SplitBoundingSectors(const std::vector<LatLon>& locations)
{
	if (locations.size() == 0)
		return std::vector<Sector>();

	double minLat = 90.0;
	double minLon = 180.0;
	double maxLat = -90.0;
	double maxLon = -180.0;

	LatLon lastLocation;
	bool hasLastLocation = false;

	for (LatLon ll : locations)
	{
		double lat = ll.Latitude().Degrees();
		if (lat < minLat)
			minLat = lat;
		if (lat > maxLat)
			maxLat = lat;

		double lon = ll.Longitude().Degrees();
		if (lon >= 0 && lon < minLon)
			minLon = lon;
		if (lon <= 0 && lon > maxLon)
			maxLon = lon;

		if (hasLastLocation)
		{
			double lastLon = lastLocation.Longitude().Degrees();
			if (Math::sign(lon) != Math::sign(lastLon))
			{
				if (Math::abs(lon - lastLon) < 180.0)
				{
					// Crossing the zero longitude line too
					maxLon = 0.0;
					minLon = 0.0;
				}
			}
		}
		lastLocation = ll;
		hasLastLocation = true;
	}

	if (minLat == maxLat && minLon == maxLon)
		return std::vector<Sector>();

	std::vector<Sector> ret;
	ret.push_back(Sector::FromDegrees(minLat, maxLat, minLon, 180.0)); // Sector on eastern hemisphere.
	ret.push_back(Sector::FromDegrees(minLat, maxLat, -180.0, maxLon)); // Sector on western hemisphere.
	return ret;
}

Sector Sector::Union(const Sector& that) const
{
	Angle minLat = this->minLatitude;
	Angle maxLat = this->maxLatitude;
	Angle minLon = this->minLongitude;
	Angle maxLon = this->maxLongitude;

	if (that.minLatitude.Degrees() < this->minLatitude.Degrees())
		minLat = that.minLatitude;
	if (that.maxLatitude.Degrees() > this->maxLatitude.Degrees())
		maxLat = that.maxLatitude;
	if (that.minLongitude.Degrees() < this->minLongitude.Degrees())
		minLon = that.minLongitude;
	if (that.maxLongitude.Degrees() > this->maxLongitude.Degrees())
		maxLon = that.maxLongitude;

	return Sector(minLat, maxLat, minLon, maxLon);
}

Sector Sector::Union(const Angle& latitude, const Angle& longitude) const
{
	Angle minLat = this->minLatitude;
	Angle maxLat = this->maxLatitude;
	Angle minLon = this->minLongitude;
	Angle maxLon = this->maxLongitude;

	if (latitude.Degrees() < this->minLatitude.Degrees())
		minLat = latitude;
	if (latitude.Degrees() > this->maxLatitude.Degrees())
		maxLat = latitude;
	if (longitude.Degrees() < this->minLongitude.Degrees())
		minLon = longitude;
	if (longitude.Degrees() > this->maxLongitude.Degrees())
		maxLon = longitude;

	return Sector(minLat, maxLat, minLon, maxLon);
}

Sector Sector::Union(const Sector& sectorA, const Sector& sectorB)
{
	return sectorA.Union(sectorB);
}

Sector Sector::Union(const std::vector<Sector>& sectors)
{
	Angle minLat = Angle::FromDegrees(90.0);
	Angle maxLat = Angle::FromDegrees(-90.0);
	Angle minLon = Angle::FromDegrees(180.0);
	Angle maxLon = Angle::FromDegrees(-180.0);

	for (Sector s : sectors)
	{
		for (LatLon p : s.AsList())
		{
			if (p.Latitude().Degrees() < minLat.Degrees())
				minLat = p.Latitude();
			if (p.Latitude().Degrees() > maxLat.Degrees())
				maxLat = p.Latitude();
			if (p.Longitude().Degrees() < minLon.Degrees())
				minLon = p.Longitude();
			if (p.Longitude().Degrees() > maxLon.Degrees())
				maxLon = p.Longitude();
		}
	}

	return Sector(minLat, maxLat, minLon, maxLon);
}

bool Sector::IsWithinLatLonLimits() const
{
	return this->minLatitude.Degrees() >= -90 && this->maxLatitude.Degrees() <= 90
		&& this->minLongitude.Degrees() >= -180 && this->maxLongitude.Degrees() <= 180;
}

bool Sector::ContainsDegrees(double degreesLatitude, double degreesLongitude) const
{
	return degreesLatitude >= this->minLatitude.Degrees() && degreesLatitude <= this->maxLatitude.Degrees()
		&& degreesLongitude >= this->minLongitude.Degrees() && degreesLongitude <= this->maxLongitude.Degrees();
}

bool Sector::ContainsRadians(double radiansLatitude, double radiansLongitude) const
{
	return radiansLatitude >= this->minLatitude.Radians() && radiansLatitude <= this->maxLatitude.Radians()
		&& radiansLongitude >= this->minLongitude.Radians() && radiansLongitude <= this->maxLongitude.Radians();
}

bool Sector::Contains(const Angle& latitude, const Angle& longitude) const
{
	return ContainsDegrees(latitude.Degrees(), longitude.Degrees());
}

bool Sector::Contains(const LatLon& latLon) const
{
	return ContainsDegrees(latLon.Latitude().Degrees(), latLon.Longitude().Degrees());
}

bool Sector::Contains(const Sector& that) const
{
	// Assumes normalized angles -- [-180, 180], [-90, 90]
	if (that.minLongitude.Degrees() < this->minLongitude.Degrees())
		return false;
	if (that.maxLongitude.Degrees() > this->maxLongitude.Degrees())
		return false;
	if (that.minLatitude.Degrees() < this->minLatitude.Degrees())
		return false;
	//noinspection RedundantIfStatement
	if (that.maxLatitude.Degrees() > this->maxLatitude.Degrees())
		return false;

	return true;
}

bool Sector::Intersects(const Sector& that) const
{
	// Assumes normalized angles -- [-180, 180], [-90, 90]
	if (that.maxLongitude.Degrees() < this->minLongitude.Degrees())
		return false;
	if (that.minLongitude.Degrees() > this->maxLongitude.Degrees())
		return false;
	if (that.maxLatitude.Degrees() < this->minLatitude.Degrees())
		return false;
	//noinspection RedundantIfStatement
	if (that.minLatitude.Degrees() > this->maxLatitude.Degrees())
		return false;

	return true;
}

bool Sector::IntersectsInterior(const Sector& that) const
{
	// Assumes normalized angles -- [-180, 180], [-90, 90]
	if (that.maxLongitude.Degrees() <= this->minLongitude.Degrees())
		return false;
	if (that.minLongitude.Degrees() >= this->maxLongitude.Degrees())
		return false;
	if (that.maxLatitude.Degrees() <= this->minLatitude.Degrees())
		return false;
	//noinspection RedundantIfStatement
	if (that.minLatitude.Degrees() >= this->maxLatitude.Degrees())
		return false;

	return true;
}

bool Sector::IntersectsSegment(const LatLon& begin, const LatLon& end) const
{
	Vec3d segmentBegin = Vec3d(begin.Longitude().Degrees(), begin.Latitude().Degrees(), 0);
	Vec3d segmentEnd = Vec3d(end.Longitude().Degrees(), end.Latitude().Degrees(), 0);
	Vec3d tmp = segmentEnd - segmentBegin;
	Vec3d segmentCenter = (segmentBegin + segmentEnd) / 2.0;
	Vec3d segmentDirection = tmp;
	normalize(segmentDirection);

	double segmentExtent = length(tmp) / 2.0;

	LatLon centroid = this->Centroid();
	Vec3d boxCenter = Vec3d(centroid.Longitude().Degrees(), centroid.Latitude().Degrees(), 0);
	double boxExtentX = this->DeltaLonDegrees() / 2.0;
	double boxExtentY = this->DeltaLatDegrees() / 2.0;

	Vec3d diff = segmentCenter - boxCenter;

	if (Math::abs(diff[0]) > (boxExtentX + segmentExtent * Math::abs(segmentDirection[0])))
	{
		return false;
	}

	if (Math::abs(diff[1]) > (boxExtentY + segmentExtent * Math::abs(segmentDirection[1])))
	{
		return false;
	}

	//noinspection SuspiciousNameCombination
	Vec3d segmentPerp = Vec3d(segmentDirection[1], -segmentDirection[0], 0);

	return Math::abs(dot(segmentPerp, diff)) <=
		(boxExtentX * Math::abs(segmentPerp[0]) + boxExtentY * Math::abs(segmentPerp[1]));
}

Sector Sector::Intersection(const Sector& that) const
{
	Angle minLat, maxLat;
	minLat = (this->minLatitude.Degrees() > that.minLatitude.Degrees()) ? this->minLatitude : that.minLatitude;
	maxLat = (this->maxLatitude.Degrees() < that.maxLatitude.Degrees()) ? this->maxLatitude : that.maxLatitude;
	if (minLat.Degrees() > maxLat.Degrees())
		return Sector();

	Angle minLon, maxLon;
	minLon = (this->minLongitude.Degrees() > that.minLongitude.Degrees()) ? this->minLongitude : that.minLongitude;
	maxLon = (this->maxLongitude.Degrees() < that.maxLongitude.Degrees()) ? this->maxLongitude : that.maxLongitude;
	if (minLon.Degrees() > maxLon.Degrees())
		return Sector();

	return Sector(minLat, maxLat, minLon, maxLon);
}

Sector Sector::Intersection(const Angle& latitude, const Angle& longitude) const
{
	if (!this->Contains(latitude, longitude))
		return Sector();
	return Sector(latitude, latitude, longitude, longitude);
}

void Sector::Subdivide(Sector sectors[]) const
{
	Angle midLat = Angle::MidAngle(this->minLatitude, this->maxLatitude);
	Angle midLon = Angle::MidAngle(this->minLongitude, this->maxLongitude);

	sectors[0] = Sector(this->minLatitude, midLat, this->minLongitude, midLon);
	sectors[1] = Sector(this->minLatitude, midLat, midLon, this->maxLongitude);
	sectors[2] = Sector(midLat, this->maxLatitude, this->minLongitude, midLon);
	sectors[3] = Sector(midLat, this->maxLatitude, midLon, this->maxLongitude);
}

std::vector<Sector> Sector::Subdivide(int div) const
{
	double dLat = this->deltaLat.Degrees() / div;
	double dLon = this->deltaLon.Degrees() / div;

	std::vector<Sector> sectors(div * div);
	int idx = 0;
	for (int row = 0; row < div; row++)
	{
		for (int col = 0; col < div; col++)
		{
			sectors[idx++] = Sector::FromDegrees(
				this->minLatitude.Degrees() + dLat * row,
				this->minLatitude.Degrees() + dLat * row + dLat,
				this->minLongitude.Degrees() + dLon * col,
				this->minLongitude.Degrees() + dLon * col + dLon);
		}
	}

	return sectors;
}

void Sector::GetCorners(LatLon corners[]) const
{
	corners[0] = LatLon(this->minLatitude, this->minLongitude);
	corners[1] = LatLon(this->minLatitude, this->maxLongitude);
	corners[2] = LatLon(this->maxLatitude, this->maxLongitude);
	corners[3] = LatLon(this->maxLatitude, this->minLongitude);
}

LatLon Sector::Centroid() const
{
	Angle la = Angle::FromDegrees(0.5 * (this->maxLatitude.Degrees() + this->minLatitude.Degrees()));
	Angle lo = Angle::FromDegrees(0.5 * (this->maxLongitude.Degrees() + this->minLongitude.Degrees()));
	return LatLon(la, lo);
}

std::vector<LatLon> Sector::AsList() const
{
	std::vector<LatLon> list(4);

	list.push_back(LatLon(this->minLatitude, this->minLongitude));
	list.push_back(LatLon(this->minLatitude, this->maxLongitude));
	list.push_back(LatLon(this->maxLatitude, this->maxLongitude));
	list.push_back(LatLon(this->maxLatitude, this->minLongitude));
	
	return list;
}

bool Sector::operator==(const Sector& that) const
{
	return minLatitude == that.minLatitude
		&& minLongitude == that.minLongitude
		&& maxLatitude == that.maxLatitude
		&& maxLongitude == that.maxLongitude;
}

bool Sector::operator!=(const Sector& that) const
{
	return minLatitude != that.minLatitude
		|| minLongitude != that.minLongitude
		|| maxLatitude != that.maxLatitude
		|| maxLongitude != that.maxLongitude;
}
