#ifndef MINISIM_WEATHERDATAPOINT_H
#define MINISIM_WEATHERDATAPOINT_H

#include "SolarCar/Aerobody/VelocityVector.hpp"

struct WeatherDataPoint {
	VelocityVector wind;
	double irradiance;
	double air_temp;
	double pressure;
	double air_density;
	double reciprocal_speed_of_sound;

	// define an average operator
	static WeatherDataPoint average(const WeatherDataPoint& rhs, const WeatherDataPoint& lhs);
};

#endif  // MINISIM_WEATHERDATAPOINT_H