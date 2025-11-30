#ifndef MINISIM_GEOGRAPHICALCOORDINATE_H
#define MINISIM_GEOGRAPHICALCOORDINATE_H

struct GeographicalCoordinate {
	double latitude;
	double longitude;

	/// Average two GeographicalCoordinates
	static GeographicalCoordinate average(const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs);
};

/// overload addition operator for GeographicalCoordinate
GeographicalCoordinate operator+(const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs);
/// overload subtraction operator for GeographicalCoordinate
GeographicalCoordinate operator-(const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs);
/// overload multiplication operator for GeographicalCoordinate
GeographicalCoordinate operator*(const GeographicalCoordinate& lhs, double rhs);
/// overload multiplication operator for GeographicalCoordinate
GeographicalCoordinate operator*(double& lhs, const GeographicalCoordinate& rhs);
/// overload division operator for GeographicalCoordinate
GeographicalCoordinate operator/(const GeographicalCoordinate& lhs, double rhs);

#endif  // MINISIM_GEOGRAPHICALCOORDINATE_H
