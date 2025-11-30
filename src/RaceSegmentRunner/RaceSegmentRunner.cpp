#include "RaceSegmentRunner.h"

#include <cmath>

double RaceSegmentRunner::calculate_resistive_force(
	const RouteSegment& route_segment, const WeatherDataPoint& weather_data, double speed) const {

	 
	 
	 
	double tire_load = (car.mass / 3.0) * route_segment.gravity;
	double rolling_resistance = 3 * car.tire.rolling_resistance(tire_load, speed);

	 
	 
	VelocityVector car_velocity = VelocityVector::from_polar_components(speed, route_segment.heading);

	 
	ApparentWindVector apparent_wind = Aerobody::get_wind(weather_data.wind, car_velocity);

	 
	double aero_drag = car.aerobody.aerodynamic_drag(apparent_wind, weather_data.air_density);

	 
	double gravitational_force = car.mass * route_segment.gravity_times_sine_road_incline_angle;

	 
	return rolling_resistance + aero_drag + gravitational_force;
}

double RaceSegmentRunner::calculate_power_out(
	const RouteSegment& route_segment, const WeatherDataPoint& weather_data, double speed) const {

	 
	double resistive_force = calculate_resistive_force(route_segment, weather_data, speed);

	 
	double angular_speed = speed / car.wheel_radius;

	 
	double torque = resistive_force * car.wheel_radius;

	 
	double motor_power = car.motor.power_consumed(angular_speed, torque);

	return motor_power;
}

double RaceSegmentRunner::calculate_power_in(
	const RouteSegment& route_segment, const WeatherDataPoint& weather_data) const {

	 
	(void)route_segment;   
	return car.array.power_in(weather_data.irradiance);
}

std::optional<double> RaceSegmentRunner::calculate_power_net(
	const RouteSegment& route_segment, const WeatherDataPoint& weather_data,
	double state_of_charge, double speed) const {

	 
	double power_in = calculate_power_in(route_segment, weather_data);
	double power_out = calculate_power_out(route_segment, weather_data, speed);

	 
	double net_power_demanded = power_out - power_in;

	 
	auto battery_loss = car.battery.power_loss(net_power_demanded, state_of_charge);

	 
	if (!battery_loss.has_value()) {
		return std::nullopt;
	}

	 
	 
	 
	double net_power = net_power_demanded + battery_loss.value();

	 
	return -net_power;
}
