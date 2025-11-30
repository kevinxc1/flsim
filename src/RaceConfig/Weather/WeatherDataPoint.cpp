#include "WeatherDataPoint.h"

WeatherDataPoint WeatherDataPoint::average(const WeatherDataPoint& rhs, const WeatherDataPoint& lhs) {
	return WeatherDataPoint{
		.wind = VelocityVector::from_cartesian_components(                   
			(rhs.wind.get_north_south() + lhs.wind.get_north_south()) / 2,   
			(rhs.wind.get_east_west() + lhs.wind.get_east_west()) / 2        
			),
		.irradiance = (rhs.irradiance + lhs.irradiance) / 2,
		.air_temp = (rhs.air_temp + lhs.air_temp) / 2,
		.pressure = (rhs.pressure + lhs.pressure) / 2,
		.air_density = (rhs.air_density + lhs.air_density) / 2,
		.reciprocal_speed_of_sound = (rhs.reciprocal_speed_of_sound + lhs.reciprocal_speed_of_sound) / 2,
	};
}
