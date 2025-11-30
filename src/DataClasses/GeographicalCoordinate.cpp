#include "GeographicalCoordinate.h"

 
GeographicalCoordinate operator+(const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs) {
	return {lhs.latitude + rhs.latitude, lhs.longitude + rhs.longitude};
}

 
GeographicalCoordinate operator-(const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs) {
	return {lhs.latitude - rhs.latitude, lhs.longitude - rhs.longitude};
}

 
GeographicalCoordinate operator*(const GeographicalCoordinate& lhs, double rhs) {
	return {lhs.latitude * rhs, lhs.longitude * rhs};
}

 
GeographicalCoordinate operator*(double& lhs, const GeographicalCoordinate& rhs) {
	return {lhs * rhs.latitude, lhs * rhs.longitude};
}

 
GeographicalCoordinate operator/(const GeographicalCoordinate& lhs, double rhs) {
	return {lhs.latitude / rhs, lhs.longitude / rhs};
}

 
GeographicalCoordinate GeographicalCoordinate::average(
	const GeographicalCoordinate& lhs, const GeographicalCoordinate& rhs) {
	return (lhs + rhs) / 2;
}
