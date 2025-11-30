#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "RaceSegmentRunner.h"

#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Route/RouteSegment.h"
#include "RaceConfig/Weather/WeatherDataPoint.h"
#include "SolarCar/SolarCar.h"
#include "SolarCar/Aerobody/Aerobody.h"
#include "SolarCar/Array/Array.h"
#include "SolarCar/Battery/Battery.h"
#include "SolarCar/Motor/Motor.h"
#include "SolarCar/Tire/Tire.h"

constexpr double EPSILON = 0.001;  

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("RaceSegmentRunner: calculate_resistive_force", "[RaceSegmentRunner]") {
	SECTION("Random Test 0") {
		constexpr double drag_coefficient = 0.00541143;
		constexpr double frontal_area = 3.42548;
		constexpr double array_area = 4.63645;
		constexpr double array_efficiency = 22.3886;
		constexpr double energy_capacity = 6105.03;
		constexpr double min_voltage = 71.3779;
		constexpr double max_voltage = 148.606;
		constexpr double resistance = 0.660223;
		constexpr double hysteresis_loss = 2.86961;
		constexpr double eddy_current_loss_coefficient = 0.00171711;
		constexpr double alpha = -8.77003;
		constexpr double beta = 7.68916;
		constexpr double a = 5.65872;
		constexpr double b = -7.02049e-06;
		constexpr double c = 0.175593;
		constexpr double pressure_at_stc = 181.903;
		constexpr double mass = 159.339;
		constexpr double wheel_radius = 0.374048;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {57.0574, -15.5825};
			constexpr GeographicalCoordinate end_coordinate = {88.342, 133.793};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 5.54993;
			constexpr double weather_station = 5.87616;
			constexpr double distance = 67.3111;
			constexpr double heading = 5.18201;
			constexpr double elevation = 425.319;
			constexpr double grade = -0.0394923;
			constexpr double road_incline_angle = -0.403056;
			constexpr double sine_road_incline_angle = -0.392231;
			constexpr double gravity = 9.80449;
			constexpr double gravity_times_sine_road_incline_angle = -3.84563;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 13.8307;
			constexpr double wind_direction = 3.90525;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 215.042;
			constexpr double air_temp = -16.1146;
			constexpr double pressure = 1056.43;
			constexpr double air_density = 1.20163;
			constexpr double reciprocal_speed_of_sound = 0.00297626;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 19.459;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 29945.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-52.0602, 84.2023};
			constexpr GeographicalCoordinate end_coordinate = {42.4756, 83.5407};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 15.8033;
			constexpr double weather_station = 2.51218;
			constexpr double distance = 10.4701;
			constexpr double heading = 1.27157;
			constexpr double elevation = -50.4694;
			constexpr double grade = 0.265976;
			constexpr double road_incline_angle = -0.836374;
			constexpr double sine_road_incline_angle = -0.742218;
			constexpr double gravity = 9.80855;
			constexpr double gravity_times_sine_road_incline_angle = -7.28008;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 38.5028;
			constexpr double wind_direction = 1.89896;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 996.597;
			constexpr double air_temp = -17.6059;
			constexpr double pressure = 903.181;
			constexpr double air_density = 1.25152;
			constexpr double reciprocal_speed_of_sound = 0.00291488;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 6.12308;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2071.02;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-41.3563, -133.388};
			constexpr GeographicalCoordinate end_coordinate = {74.691, 77.703};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 23.5566;
			constexpr double weather_station = 2.08289;
			constexpr double distance = 57.7646;
			constexpr double heading = 5.70869;
			constexpr double elevation = 335.466;
			constexpr double grade = 0.0185615;
			constexpr double road_incline_angle = 1.11229;
			constexpr double sine_road_incline_angle = 0.896714;
			constexpr double gravity = 9.79368;
			constexpr double gravity_times_sine_road_incline_angle = 8.78213;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 45.7953;
			constexpr double wind_direction = 3.48038;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 720.105;
			constexpr double air_temp = -27.026;
			constexpr double pressure = 1022.78;
			constexpr double air_density = 1.1901;
			constexpr double reciprocal_speed_of_sound = 0.00303442;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 15.2015;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 19965.9;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 1") {
		constexpr double drag_coefficient = 0.00267739;
		constexpr double frontal_area = 4.84856;
		constexpr double array_area = 4.95817;
		constexpr double array_efficiency = 24.9941;
		constexpr double energy_capacity = 1512.92;
		constexpr double min_voltage = 91.1046;
		constexpr double max_voltage = 124.383;
		constexpr double resistance = 0.580563;
		constexpr double hysteresis_loss = 1.0432;
		constexpr double eddy_current_loss_coefficient = 0.0415655;
		constexpr double alpha = 1.31118;
		constexpr double beta = -4.68206;
		constexpr double a = -5.15711;
		constexpr double b = 7.41434e-06;
		constexpr double c = -0.0643793;
		constexpr double pressure_at_stc = 138.285;
		constexpr double mass = 871.681;
		constexpr double wheel_radius = 0.266795;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-15.1774, -157.28};
			constexpr GeographicalCoordinate end_coordinate = {-63.5884, 129.811};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 164.66;
			constexpr double weather_station = 2.39927;
			constexpr double distance = 27.328;
			constexpr double heading = 0.139727;
			constexpr double elevation = -288.07;
			constexpr double grade = 0.0913732;
			constexpr double road_incline_angle = 1.4573;
			constexpr double sine_road_incline_angle = 0.993566;
			constexpr double gravity = 9.79046;
			constexpr double gravity_times_sine_road_incline_angle = 9.72747;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 19.3939;
			constexpr double wind_direction = 4.23081;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 682.946;
			constexpr double air_temp = -18.3736;
			constexpr double pressure = 912.368;
			constexpr double air_density = 1.21198;
			constexpr double reciprocal_speed_of_sound = 0.00304971;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 18.2836;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 8479.64;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-51.8529, -63.5593};
			constexpr GeographicalCoordinate end_coordinate = {-34.9415, 120.544};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 191.546;
			constexpr double weather_station = 8.08687;
			constexpr double distance = 57.7606;
			constexpr double heading = 0.528921;
			constexpr double elevation = 323.675;
			constexpr double grade = 0.848978;
			constexpr double road_incline_angle = -0.193808;
			constexpr double sine_road_incline_angle = -0.192597;
			constexpr double gravity = 9.80178;
			constexpr double gravity_times_sine_road_incline_angle = -1.8878;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 25.4602;
			constexpr double wind_direction = 4.2004;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 168.917;
			constexpr double air_temp = -24.9295;
			constexpr double pressure = 1054.15;
			constexpr double air_density = 1.18747;
			constexpr double reciprocal_speed_of_sound = 0.00302951;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 13.3648;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1644.99;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-2.43649, 173.453};
			constexpr GeographicalCoordinate end_coordinate = {-69.9507, -143.795};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 15.1257;
			constexpr double weather_station = 4.68595;
			constexpr double distance = 5.48059;
			constexpr double heading = 3.84563;
			constexpr double elevation = -321.165;
			constexpr double grade = 0.184042;
			constexpr double road_incline_angle = 1.01823;
			constexpr double sine_road_incline_angle = 0.85118;
			constexpr double gravity = 9.78436;
			constexpr double gravity_times_sine_road_incline_angle = 8.32824;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 44.7904;
			constexpr double wind_direction = 2.30881;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 840.569;
			constexpr double air_temp = 5.23253;
			constexpr double pressure = 999.522;
			constexpr double air_density = 1.03117;
			constexpr double reciprocal_speed_of_sound = 0.00305276;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 10.5377;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 7260.55;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 2") {
		constexpr double drag_coefficient = 0.00222216;
		constexpr double frontal_area = 2.52092;
		constexpr double array_area = 7.76055;
		constexpr double array_efficiency = 25.4659;
		constexpr double energy_capacity = 2339.46;
		constexpr double min_voltage = 72.7279;
		constexpr double max_voltage = 128.986;
		constexpr double resistance = 0.614703;
		constexpr double hysteresis_loss = 4.58069;
		constexpr double eddy_current_loss_coefficient = 0.0437772;
		constexpr double alpha = -7.00712;
		constexpr double beta = 6.4752;
		constexpr double a = 9.93578;
		constexpr double b = 3.24632e-06;
		constexpr double c = 0.938711;
		constexpr double pressure_at_stc = 181.47;
		constexpr double mass = 685.134;
		constexpr double wheel_radius = 0.188291;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-52.2123, 100.073};
			constexpr GeographicalCoordinate end_coordinate = {-83.7832, -175.094};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 108.491;
			constexpr double weather_station = 9.07849;
			constexpr double distance = 8.2932;
			constexpr double heading = 4.50096;
			constexpr double elevation = 379.924;
			constexpr double grade = 0.0622978;
			constexpr double road_incline_angle = 0.115196;
			constexpr double sine_road_incline_angle = 0.114942;
			constexpr double gravity = 9.80124;
			constexpr double gravity_times_sine_road_incline_angle = 1.12657;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 2.5212;
			constexpr double wind_direction = 1.98963;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 664.438;
			constexpr double air_temp = -11.6514;
			constexpr double pressure = 1031.66;
			constexpr double air_density = 1.07357;
			constexpr double reciprocal_speed_of_sound = 0.00307251;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 3.42966;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 3.3552e+08;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-45.1936, 155.17};
			constexpr GeographicalCoordinate end_coordinate = {-61.0774, -116.769};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 37.3546;
			constexpr double weather_station = 1.7007;
			constexpr double distance = 51.5937;
			constexpr double heading = 1.24286;
			constexpr double elevation = 471.045;
			constexpr double grade = -0.618307;
			constexpr double road_incline_angle = 0.157498;
			constexpr double sine_road_incline_angle = 0.156848;
			constexpr double gravity = 9.79248;
			constexpr double gravity_times_sine_road_incline_angle = 1.53593;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 37.283;
			constexpr double wind_direction = 4.03827;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 585.029;
			constexpr double air_temp = -17.3312;
			constexpr double pressure = 959.886;
			constexpr double air_density = 1.29486;
			constexpr double reciprocal_speed_of_sound = 0.00292474;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 6.26055;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 1.06103e+09;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {20.1168, 89.1229};
			constexpr GeographicalCoordinate end_coordinate = {71.3468, -142.728};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 197.966;
			constexpr double weather_station = 9.48864;
			constexpr double distance = 80.3611;
			constexpr double heading = 2.90715;
			constexpr double elevation = -429.11;
			constexpr double grade = -0.383874;
			constexpr double road_incline_angle = -0.367448;
			constexpr double sine_road_incline_angle = -0.359235;
			constexpr double gravity = 9.81219;
			constexpr double gravity_times_sine_road_incline_angle = -3.52488;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 12.6353;
			constexpr double wind_direction = 2.04224;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 319.48;
			constexpr double air_temp = -19.04;
			constexpr double pressure = 956.569;
			constexpr double air_density = 1.20644;
			constexpr double reciprocal_speed_of_sound = 0.00302922;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 8.9899;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2.19319e+09;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 3") {
		constexpr double drag_coefficient = 0.00692069;
		constexpr double frontal_area = 3.64819;
		constexpr double array_area = 5.58087;
		constexpr double array_efficiency = 27.102;
		constexpr double energy_capacity = 6951.37;
		constexpr double min_voltage = 70.9177;
		constexpr double max_voltage = 135.16;
		constexpr double resistance = 0.498703;
		constexpr double hysteresis_loss = 4.48842;
		constexpr double eddy_current_loss_coefficient = 0.0394493;
		constexpr double alpha = 4.33011;
		constexpr double beta = -9.87724;
		constexpr double a = -7.42584;
		constexpr double b = -4.9009e-06;
		constexpr double c = 0.368808;
		constexpr double pressure_at_stc = 137.704;
		constexpr double mass = 249.93;
		constexpr double wheel_radius = 0.267808;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-34.7195, 141.789};
			constexpr GeographicalCoordinate end_coordinate = {-30.8653, -66.9865};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 37.3154;
			constexpr double weather_station = 4.04947;
			constexpr double distance = 39.9317;
			constexpr double heading = 0.153082;
			constexpr double elevation = -28.5721;
			constexpr double grade = -0.910287;
			constexpr double road_incline_angle = 1.16433;
			constexpr double sine_road_incline_angle = 0.918523;
			constexpr double gravity = 9.80771;
			constexpr double gravity_times_sine_road_incline_angle = 9.0086;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 2.3359;
			constexpr double wind_direction = 2.13384;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 977.402;
			constexpr double air_temp = -0.274937;
			constexpr double pressure = 920.826;
			constexpr double air_density = 1.05921;
			constexpr double reciprocal_speed_of_sound = 0.00307115;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 6.04288;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2251.87;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {26.4084, 16.9736};
			constexpr GeographicalCoordinate end_coordinate = {33.3134, -138.136};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 170.158;
			constexpr double weather_station = 0.322006;
			constexpr double distance = 13.4364;
			constexpr double heading = 4.82226;
			constexpr double elevation = 50.3591;
			constexpr double grade = 0.177831;
			constexpr double road_incline_angle = 1.27472;
			constexpr double sine_road_incline_angle = 0.956489;
			constexpr double gravity = 9.79766;
			constexpr double gravity_times_sine_road_incline_angle = 9.37135;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 10.5152;
			constexpr double wind_direction = 5.96733;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 326.563;
			constexpr double air_temp = 42.3263;
			constexpr double pressure = 1085.19;
			constexpr double air_density = 1.04416;
			constexpr double reciprocal_speed_of_sound = 0.00292496;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 18.1454;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2348.85;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-63.1249, 85.2418};
			constexpr GeographicalCoordinate end_coordinate = {-54.9719, 149.569};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 172.287;
			constexpr double weather_station = 5.8597;
			constexpr double distance = 52.2275;
			constexpr double heading = 0.277685;
			constexpr double elevation = -183.855;
			constexpr double grade = 0.84854;
			constexpr double road_incline_angle = 1.24643;
			constexpr double sine_road_incline_angle = 0.947852;
			constexpr double gravity = 9.78106;
			constexpr double gravity_times_sine_road_incline_angle = 9.271;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 28.3094;
			constexpr double wind_direction = 2.81419;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 871.685;
			constexpr double air_temp = 35.9183;
			constexpr double pressure = 1009.34;
			constexpr double air_density = 1.16261;
			constexpr double reciprocal_speed_of_sound = 0.00299728;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 24.2198;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2317.11;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 4") {
		constexpr double drag_coefficient = 0.00159835;
		constexpr double frontal_area = 5.14783;
		constexpr double array_area = 5.16622;
		constexpr double array_efficiency = 26.2075;
		constexpr double energy_capacity = 4145.6;
		constexpr double min_voltage = 116.787;
		constexpr double max_voltage = 156.34;
		constexpr double resistance = 0.429983;
		constexpr double hysteresis_loss = 4.00419;
		constexpr double eddy_current_loss_coefficient = 0.0499121;
		constexpr double alpha = -2.40795;
		constexpr double beta = 6.00575;
		constexpr double a = 5.44814;
		constexpr double b = -4.3662e-07;
		constexpr double c = -0.4239;
		constexpr double pressure_at_stc = 199.837;
		constexpr double mass = 502.551;
		constexpr double wheel_radius = 0.235559;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {29.3269, 166.593};
			constexpr GeographicalCoordinate end_coordinate = {11.546, -11.1153};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 163.385;
			constexpr double weather_station = 4.99704;
			constexpr double distance = 50.19;
			constexpr double heading = 2.1233;
			constexpr double elevation = -247.559;
			constexpr double grade = 0.970761;
			constexpr double road_incline_angle = -0.405119;
			constexpr double sine_road_incline_angle = -0.394128;
			constexpr double gravity = 9.78377;
			constexpr double gravity_times_sine_road_incline_angle = -3.85606;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 31.8415;
			constexpr double wind_direction = 3.89352;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 200.568;
			constexpr double air_temp = -1.90299;
			constexpr double pressure = 1087.97;
			constexpr double air_density = 1.00504;
			constexpr double reciprocal_speed_of_sound = 0.00294616;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 3.6515;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1.18672e+16;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {51.0852, -62.2124};
			constexpr GeographicalCoordinate end_coordinate = {29.6903, 117.899};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 171.058;
			constexpr double weather_station = 7.68139;
			constexpr double distance = 5.09915;
			constexpr double heading = 4.53663;
			constexpr double elevation = 127.26;
			constexpr double grade = -0.517401;
			constexpr double road_incline_angle = -0.850806;
			constexpr double sine_road_incline_angle = -0.751812;
			constexpr double gravity = 9.80487;
			constexpr double gravity_times_sine_road_incline_angle = -7.37142;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 29.6569;
			constexpr double wind_direction = 2.64771;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 644.764;
			constexpr double air_temp = -30.1869;
			constexpr double pressure = 1041.56;
			constexpr double air_density = 1.09147;
			constexpr double reciprocal_speed_of_sound = 0.00301359;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 14.3665;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -2.0008e+17;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {17.1312, 22.585};
			constexpr GeographicalCoordinate end_coordinate = {-13.6682, -38.8767};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 93.5797;
			constexpr double weather_station = 5.01309;
			constexpr double distance = 20.7453;
			constexpr double heading = 5.28047;
			constexpr double elevation = -236.475;
			constexpr double grade = -0.652204;
			constexpr double road_incline_angle = 0.813662;
			constexpr double sine_road_incline_angle = 0.726807;
			constexpr double gravity = 9.80479;
			constexpr double gravity_times_sine_road_incline_angle = 7.12619;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 46.8245;
			constexpr double wind_direction = 0.943243;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 921.31;
			constexpr double air_temp = 6.72691;
			constexpr double pressure = 1001.81;
			constexpr double air_density = 1.09559;
			constexpr double reciprocal_speed_of_sound = 0.00295147;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 3.08203;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -8.28625e+15;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 5") {
		constexpr double drag_coefficient = 0.00168911;
		constexpr double frontal_area = 8.17636;
		constexpr double array_area = 9.97561;
		constexpr double array_efficiency = 28.4217;
		constexpr double energy_capacity = 2777.89;
		constexpr double min_voltage = 143.082;
		constexpr double max_voltage = 145.763;
		constexpr double resistance = 0.0748078;
		constexpr double hysteresis_loss = 4.78703;
		constexpr double eddy_current_loss_coefficient = 0.0353913;
		constexpr double alpha = 8.39894;
		constexpr double beta = -3.53853;
		constexpr double a = -9.5303;
		constexpr double b = -2.17734e-06;
		constexpr double c = 0.311488;
		constexpr double pressure_at_stc = 131.857;
		constexpr double mass = 210.11;
		constexpr double wheel_radius = 0.363856;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-39.4474, 122.316};
			constexpr GeographicalCoordinate end_coordinate = {12.9534, 129.264};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 58.621;
			constexpr double weather_station = 2.18641;
			constexpr double distance = 62.4266;
			constexpr double heading = 1.36105;
			constexpr double elevation = -194.998;
			constexpr double grade = 0.0442188;
			constexpr double road_incline_angle = 0.34381;
			constexpr double sine_road_incline_angle = 0.337076;
			constexpr double gravity = 9.78744;
			constexpr double gravity_times_sine_road_incline_angle = 3.29911;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 42.1264;
			constexpr double wind_direction = 5.37201;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 168.111;
			constexpr double air_temp = 30.3168;
			constexpr double pressure = 967.157;
			constexpr double air_density = 1.1047;
			constexpr double reciprocal_speed_of_sound = 0.0029631;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 24.2251;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 4.18117e+11;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {48.1784, 52.6767};
			constexpr GeographicalCoordinate end_coordinate = {70.5255, -105.715};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 152.436;
			constexpr double weather_station = 9.9093;
			constexpr double distance = 64.3998;
			constexpr double heading = 4.15627;
			constexpr double elevation = 25.2417;
			constexpr double grade = 0.902328;
			constexpr double road_incline_angle = -0.492245;
			constexpr double sine_road_incline_angle = -0.472605;
			constexpr double gravity = 9.80946;
			constexpr double gravity_times_sine_road_incline_angle = -4.636;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 11.1119;
			constexpr double wind_direction = 4.29605;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 23.5358;
			constexpr double air_temp = -47.969;
			constexpr double pressure = 914.819;
			constexpr double air_density = 1.18594;
			constexpr double reciprocal_speed_of_sound = 0.00301554;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.6597;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 4.65594e+11;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {57.3587, -52.6388};
			constexpr GeographicalCoordinate end_coordinate = {81.0932, 7.49623};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 130.238;
			constexpr double weather_station = 3.40906;
			constexpr double distance = 76.0923;
			constexpr double heading = 5.27459;
			constexpr double elevation = -476.354;
			constexpr double grade = 0.741871;
			constexpr double road_incline_angle = 0.51731;
			constexpr double sine_road_incline_angle = 0.494544;
			constexpr double gravity = 9.80193;
			constexpr double gravity_times_sine_road_incline_angle = 4.84748;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 19.5211;
			constexpr double wind_direction = 0.229084;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 98.6617;
			constexpr double air_temp = -10.6259;
			constexpr double pressure = 931.31;
			constexpr double air_density = 1.28139;
			constexpr double reciprocal_speed_of_sound = 0.00291581;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 19.2925;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 2.63182e+11;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 6") {
		constexpr double drag_coefficient = 0.0073156;
		constexpr double frontal_area = 9.68362;
		constexpr double array_area = 5.33245;
		constexpr double array_efficiency = 29.9222;
		constexpr double energy_capacity = 1487.08;
		constexpr double min_voltage = 129.567;
		constexpr double max_voltage = 152.06;
		constexpr double resistance = 0.875191;
		constexpr double hysteresis_loss = 4.39467;
		constexpr double eddy_current_loss_coefficient = 0.00320175;
		constexpr double alpha = -4.3041;
		constexpr double beta = 2.9667;
		constexpr double a = -4.28558;
		constexpr double b = 1.45276e-07;
		constexpr double c = -0.661424;
		constexpr double pressure_at_stc = 181.878;
		constexpr double mass = 605.638;
		constexpr double wheel_radius = 0.215294;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {3.46655, 178.735};
			constexpr GeographicalCoordinate end_coordinate = {-42.6708, -109.717};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 65.7339;
			constexpr double weather_station = 0.355442;
			constexpr double distance = 7.89692;
			constexpr double heading = 3.93579;
			constexpr double elevation = 420.716;
			constexpr double grade = -0.409725;
			constexpr double road_incline_angle = 1.5135;
			constexpr double sine_road_incline_angle = 0.998359;
			constexpr double gravity = 9.81036;
			constexpr double gravity_times_sine_road_incline_angle = 9.79426;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 8.77508;
			constexpr double wind_direction = 0.12804;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 352.636;
			constexpr double air_temp = 36.502;
			constexpr double pressure = 974.538;
			constexpr double air_density = 1.04762;
			constexpr double reciprocal_speed_of_sound = 0.0030755;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 21.0589;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -6995.98;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-53.904, 103.24};
			constexpr GeographicalCoordinate end_coordinate = {60.5743, -163.604};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 102.171;
			constexpr double weather_station = 4.5468;
			constexpr double distance = 39.1106;
			constexpr double heading = 0.898491;
			constexpr double elevation = 403.282;
			constexpr double grade = -0.0537783;
			constexpr double road_incline_angle = 1.22549;
			constexpr double sine_road_incline_angle = 0.940972;
			constexpr double gravity = 9.81974;
			constexpr double gravity_times_sine_road_incline_angle = 9.2401;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 20.4268;
			constexpr double wind_direction = 4.79254;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 729.937;
			constexpr double air_temp = 4.01076;
			constexpr double pressure = 1060.66;
			constexpr double air_density = 1.05076;
			constexpr double reciprocal_speed_of_sound = 0.00304929;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.0364;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -12728.7;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-50.3778, -138.891};
			constexpr GeographicalCoordinate end_coordinate = {-60.0831, 168.431};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 171.139;
			constexpr double weather_station = 8.99916;
			constexpr double distance = 81.4888;
			constexpr double heading = 5.6128;
			constexpr double elevation = 222.466;
			constexpr double grade = -0.312128;
			constexpr double road_incline_angle = -1.10073;
			constexpr double sine_road_incline_angle = -0.891539;
			constexpr double gravity = 9.81312;
			constexpr double gravity_times_sine_road_incline_angle = -8.74878;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 3.16849;
			constexpr double wind_direction = 2.79779;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 933.002;
			constexpr double air_temp = 34.7927;
			constexpr double pressure = 1048.61;
			constexpr double air_density = 1.00154;
			constexpr double reciprocal_speed_of_sound = 0.00305062;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.8179;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -24731;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 7") {
		constexpr double drag_coefficient = 0.00750978;
		constexpr double frontal_area = 1.74717;
		constexpr double array_area = 5.87214;
		constexpr double array_efficiency = 17.5686;
		constexpr double energy_capacity = 7188.01;
		constexpr double min_voltage = 88.7963;
		constexpr double max_voltage = 94.3098;
		constexpr double resistance = 0.7694;
		constexpr double hysteresis_loss = 3.39415;
		constexpr double eddy_current_loss_coefficient = 0.0125603;
		constexpr double alpha = -9.27924;
		constexpr double beta = 7.30799;
		constexpr double a = 9.55063;
		constexpr double b = 3.59488e-06;
		constexpr double c = -0.319232;
		constexpr double pressure_at_stc = 195.833;
		constexpr double mass = 160.493;
		constexpr double wheel_radius = 0.423374;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-73.835, 177.821};
			constexpr GeographicalCoordinate end_coordinate = {-87.6007, -178.024};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 60.5313;
			constexpr double weather_station = 6.85325;
			constexpr double distance = 90.2911;
			constexpr double heading = 5.06421;
			constexpr double elevation = -291.236;
			constexpr double grade = -0.550221;
			constexpr double road_incline_angle = -1.31724;
			constexpr double sine_road_incline_angle = -0.968025;
			constexpr double gravity = 9.81425;
			constexpr double gravity_times_sine_road_incline_angle = -9.50044;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.1269;
			constexpr double wind_direction = 0.398954;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 534.449;
			constexpr double air_temp = -23.6623;
			constexpr double pressure = 1017.65;
			constexpr double air_density = 1.01427;
			constexpr double reciprocal_speed_of_sound = 0.00295032;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 20.4469;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1733.69;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {61.2949, -80.895};
			constexpr GeographicalCoordinate end_coordinate = {46.7371, -116.268};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 19.2451;
			constexpr double weather_station = 4.21877;
			constexpr double distance = 60.4501;
			constexpr double heading = 5.7753;
			constexpr double elevation = 243.985;
			constexpr double grade = 0.485829;
			constexpr double road_incline_angle = -1.26555;
			constexpr double sine_road_incline_angle = -0.953773;
			constexpr double gravity = 9.80681;
			constexpr double gravity_times_sine_road_incline_angle = -9.35348;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 44.638;
			constexpr double wind_direction = 0.0466104;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 465.562;
			constexpr double air_temp = 19.0613;
			constexpr double pressure = 963.588;
			constexpr double air_density = 1.16296;
			constexpr double reciprocal_speed_of_sound = 0.00290887;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 21.2017;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1700.47;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {68.781, -74.3643};
			constexpr GeographicalCoordinate end_coordinate = {62.9998, 158.457};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 99.4627;
			constexpr double weather_station = 0.440786;
			constexpr double distance = 88.8585;
			constexpr double heading = 5.04138;
			constexpr double elevation = -13.296;
			constexpr double grade = 0.941749;
			constexpr double road_incline_angle = -1.56285;
			constexpr double sine_road_incline_angle = -0.999968;
			constexpr double gravity = 9.79994;
			constexpr double gravity_times_sine_road_incline_angle = -9.79963;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 16.1024;
			constexpr double wind_direction = 1.63476;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 312.367;
			constexpr double air_temp = 28.7678;
			constexpr double pressure = 1066.84;
			constexpr double air_density = 1.29285;
			constexpr double reciprocal_speed_of_sound = 0.00292689;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 21.9284;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1813.02;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 8") {
		constexpr double drag_coefficient = 0.0012743;
		constexpr double frontal_area = 2.8286;
		constexpr double array_area = 6.97849;
		constexpr double array_efficiency = 27.3159;
		constexpr double energy_capacity = 2028.65;
		constexpr double min_voltage = 73.834;
		constexpr double max_voltage = 149.641;
		constexpr double resistance = 0.839359;
		constexpr double hysteresis_loss = 1.37359;
		constexpr double eddy_current_loss_coefficient = 0.0447885;
		constexpr double alpha = 8.72353;
		constexpr double beta = 1.83711;
		constexpr double a = 2.31352;
		constexpr double b = 6.0429e-06;
		constexpr double c = 0.555203;
		constexpr double pressure_at_stc = 170.975;
		constexpr double mass = 931.226;
		constexpr double wheel_radius = 0.474491;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-70.1584, -97.3274};
			constexpr GeographicalCoordinate end_coordinate = {70.5967, -72.3697};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 92.5232;
			constexpr double weather_station = 8.25441;
			constexpr double distance = 32.4665;
			constexpr double heading = 0.0868312;
			constexpr double elevation = 182.787;
			constexpr double grade = 0.263627;
			constexpr double road_incline_angle = 0.10891;
			constexpr double sine_road_incline_angle = 0.108695;
			constexpr double gravity = 9.80646;
			constexpr double gravity_times_sine_road_incline_angle = 1.06591;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 30.2616;
			constexpr double wind_direction = 3.41843;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 162.334;
			constexpr double air_temp = 36.7537;
			constexpr double pressure = 990.834;
			constexpr double air_density = 1.27019;
			constexpr double reciprocal_speed_of_sound = 0.00298326;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.2987;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 1.04503e+30;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {53.0359, 152.353};
			constexpr GeographicalCoordinate end_coordinate = {69.7375, -163.851};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 50.8415;
			constexpr double weather_station = 1.8886;
			constexpr double distance = 84.2512;
			constexpr double heading = 2.10067;
			constexpr double elevation = 45.3925;
			constexpr double grade = -0.74012;
			constexpr double road_incline_angle = -1.26297;
			constexpr double sine_road_incline_angle = -0.952995;
			constexpr double gravity = 9.78469;
			constexpr double gravity_times_sine_road_incline_angle = -9.32476;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 45.4727;
			constexpr double wind_direction = 3.79994;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 811.469;
			constexpr double air_temp = 31.4893;
			constexpr double pressure = 935.394;
			constexpr double air_density = 1.07631;
			constexpr double reciprocal_speed_of_sound = 0.00306697;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 27.8025;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 1.25687e+30;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {70.1221, 127.347};
			constexpr GeographicalCoordinate end_coordinate = {6.5274, 90.0996};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 102.279;
			constexpr double weather_station = 3.9968;
			constexpr double distance = 82.1624;
			constexpr double heading = 2.64465;
			constexpr double elevation = -337.899;
			constexpr double grade = -0.149776;
			constexpr double road_incline_angle = -1.09908;
			constexpr double sine_road_incline_angle = -0.89079;
			constexpr double gravity = 9.78848;
			constexpr double gravity_times_sine_road_incline_angle = -8.71948;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.149;
			constexpr double wind_direction = 3.83847;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 657.385;
			constexpr double air_temp = -49.8385;
			constexpr double pressure = 981.41;
			constexpr double air_density = 1.09567;
			constexpr double reciprocal_speed_of_sound = 0.00295209;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 19.0259;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = 5.8929e+29;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 9") {
		constexpr double drag_coefficient = 0.00709895;
		constexpr double frontal_area = 9.81454;
		constexpr double array_area = 9.72139;
		constexpr double array_efficiency = 16.2012;
		constexpr double energy_capacity = 6339.1;
		constexpr double min_voltage = 87.3955;
		constexpr double max_voltage = 101.59;
		constexpr double resistance = 0.548823;
		constexpr double hysteresis_loss = 2.76483;
		constexpr double eddy_current_loss_coefficient = 0.0479474;
		constexpr double alpha = 7.14133;
		constexpr double beta = -2.96766;
		constexpr double a = -7.9244;
		constexpr double b = 2.79501e-06;
		constexpr double c = -0.0576744;
		constexpr double pressure_at_stc = 173.387;
		constexpr double mass = 962.478;
		constexpr double wheel_radius = 0.348616;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {49.7449, -121.057};
			constexpr GeographicalCoordinate end_coordinate = {-41.4128, 132.524};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 137.51;
			constexpr double weather_station = 3.90977;
			constexpr double distance = 3.23346;
			constexpr double heading = 2.434;
			constexpr double elevation = 123.935;
			constexpr double grade = -0.710767;
			constexpr double road_incline_angle = -1.32902;
			constexpr double sine_road_incline_angle = -0.970914;
			constexpr double gravity = 9.8067;
			constexpr double gravity_times_sine_road_incline_angle = -9.52147;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 29.3805;
			constexpr double wind_direction = 2.92442;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 526.75;
			constexpr double air_temp = 35.4439;
			constexpr double pressure = 993.907;
			constexpr double air_density = 1.24796;
			constexpr double reciprocal_speed_of_sound = 0.00307716;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.3525;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -5.95843e+08;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-39.7574, 155.974};
			constexpr GeographicalCoordinate end_coordinate = {35.713, 25.3775};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 110.251;
			constexpr double weather_station = 1.71325;
			constexpr double distance = 37.6581;
			constexpr double heading = 3.00648;
			constexpr double elevation = -109.717;
			constexpr double grade = 0.297958;
			constexpr double road_incline_angle = 1.47965;
			constexpr double sine_road_incline_angle = 0.995849;
			constexpr double gravity = 9.78829;
			constexpr double gravity_times_sine_road_incline_angle = 9.74765;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 0.192324;
			constexpr double wind_direction = 1.69254;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 975.911;
			constexpr double air_temp = -40.1182;
			constexpr double pressure = 951.179;
			constexpr double air_density = 1.22506;
			constexpr double reciprocal_speed_of_sound = 0.00292093;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 17.5404;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -2.91862e+08;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {88.3394, 84.6807};
			constexpr GeographicalCoordinate end_coordinate = {44.5733, 18.6193};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 188.21;
			constexpr double weather_station = 1.87032;
			constexpr double distance = 3.73212;
			constexpr double heading = 0.409862;
			constexpr double elevation = 437.657;
			constexpr double grade = -0.593428;
			constexpr double road_incline_angle = 1.1377;
			constexpr double sine_road_incline_angle = 0.907669;
			constexpr double gravity = 9.8069;
			constexpr double gravity_times_sine_road_incline_angle = 8.90142;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 17.9712;
			constexpr double wind_direction = 5.98398;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 123.913;
			constexpr double air_temp = -45.1925;
			constexpr double pressure = 907.481;
			constexpr double air_density = 1.24738;
			constexpr double reciprocal_speed_of_sound = 0.00309;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 1.24692;

			const double result = runner.calculate_resistive_force(route_segment, weather_data, speed);
			constexpr double expected = -1.10771e+07;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
}
TEST_CASE("RaceSegmentRunner: calculate_power_out", "[RaceSegmentRunner]") {
	SECTION("Random Test 0") {
		constexpr double drag_coefficient = 0.00341362;
		constexpr double frontal_area = 7.41659;
		constexpr double array_area = 6.11546;
		constexpr double array_efficiency = 25.497;
		constexpr double energy_capacity = 7954.56;
		constexpr double min_voltage = 74.6491;
		constexpr double max_voltage = 87.2849;
		constexpr double resistance = 0.194205;
		constexpr double hysteresis_loss = 3.49068;
		constexpr double eddy_current_loss_coefficient = 0.0408636;
		constexpr double alpha = 9.05537;
		constexpr double beta = 7.45887;
		constexpr double a = -7.93573;
		constexpr double b = 2.45456e-06;
		constexpr double c = -0.661155;
		constexpr double pressure_at_stc = 100.518;
		constexpr double mass = 581.258;
		constexpr double wheel_radius = 0.467783;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-33.8781, -51.9825};
			constexpr GeographicalCoordinate end_coordinate = {-1.85779, 118.126};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 125.549;
			constexpr double weather_station = 4.1643;
			constexpr double distance = 76.9527;
			constexpr double heading = 4.37467;
			constexpr double elevation = -60.9057;
			constexpr double grade = 0.0764563;
			constexpr double road_incline_angle = 0.650206;
			constexpr double sine_road_incline_angle = 0.60535;
			constexpr double gravity = 9.79881;
			constexpr double gravity_times_sine_road_incline_angle = 5.93171;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 22.7388;
			constexpr double wind_direction = 5.2725;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 183.088;
			constexpr double air_temp = -29.3346;
			constexpr double pressure = 907.753;
			constexpr double air_density = 1.05039;
			constexpr double reciprocal_speed_of_sound = 0.0029722;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 12.6081;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.98996e+47;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-11.6193, 67.6361};
			constexpr GeographicalCoordinate end_coordinate = {80.5484, -101.009};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 6.26756;
			constexpr double weather_station = 7.7748;
			constexpr double distance = 58.9171;
			constexpr double heading = 1.83237;
			constexpr double elevation = 282.418;
			constexpr double grade = -0.944055;
			constexpr double road_incline_angle = -1.15097;
			constexpr double sine_road_incline_angle = -0.91316;
			constexpr double gravity = 9.78949;
			constexpr double gravity_times_sine_road_incline_angle = -8.93937;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 4.67931;
			constexpr double wind_direction = 0.662955;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 806.419;
			constexpr double air_temp = -25.8986;
			constexpr double pressure = 962.323;
			constexpr double air_density = 1.29374;
			constexpr double reciprocal_speed_of_sound = 0.00303764;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 27.6486;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -2.07415e+48;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {74.0795, -153.193};
			constexpr GeographicalCoordinate end_coordinate = {23.7132, -148.552};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 145.451;
			constexpr double weather_station = 8.29967;
			constexpr double distance = 59.9471;
			constexpr double heading = 1.37609;
			constexpr double elevation = -474.667;
			constexpr double grade = -0.703334;
			constexpr double road_incline_angle = -1.10758;
			constexpr double sine_road_incline_angle = -0.894619;
			constexpr double gravity = 9.78256;
			constexpr double gravity_times_sine_road_incline_angle = -8.75167;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 15.2559;
			constexpr double wind_direction = 1.36303;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 763.778;
			constexpr double air_temp = -46.509;
			constexpr double pressure = 1073.04;
			constexpr double air_density = 1.12772;
			constexpr double reciprocal_speed_of_sound = 0.0030329;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.0262;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.53048e+48;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 1") {
		constexpr double drag_coefficient = 0.00826029;
		constexpr double frontal_area = 4.45082;
		constexpr double array_area = 7.09308;
		constexpr double array_efficiency = 28.8137;
		constexpr double energy_capacity = 1965.16;
		constexpr double min_voltage = 71.9892;
		constexpr double max_voltage = 128.528;
		constexpr double resistance = 0.412343;
		constexpr double hysteresis_loss = 3.2986;
		constexpr double eddy_current_loss_coefficient = 0.0191704;
		constexpr double alpha = -4.82493;
		constexpr double beta = 6.85402;
		constexpr double a = 2.05126;
		constexpr double b = -1.75405e-06;
		constexpr double c = -0.79918;
		constexpr double pressure_at_stc = 133.187;
		constexpr double mass = 867.676;
		constexpr double wheel_radius = 0.439861;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-8.0933, -52.6988};
			constexpr GeographicalCoordinate end_coordinate = {20.2392, -129.74};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 167.53;
			constexpr double weather_station = 4.13611;
			constexpr double distance = 82.4164;
			constexpr double heading = 2.03129;
			constexpr double elevation = 273.945;
			constexpr double grade = -0.225552;
			constexpr double road_incline_angle = 1.43211;
			constexpr double sine_road_incline_angle = 0.990399;
			constexpr double gravity = 9.8037;
			constexpr double gravity_times_sine_road_incline_angle = 9.70957;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 14.6918;
			constexpr double wind_direction = 6.25187;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 528.313;
			constexpr double air_temp = 21.4893;
			constexpr double pressure = 1068.45;
			constexpr double air_density = 1.0621;
			constexpr double reciprocal_speed_of_sound = 0.00300934;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 14.5523;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -2.4817e+18;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {74.805, -67.002};
			constexpr GeographicalCoordinate end_coordinate = {-59.3521, -13.4898};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 37.487;
			constexpr double weather_station = 6.34261;
			constexpr double distance = 62.2024;
			constexpr double heading = 3.93341;
			constexpr double elevation = -40.0597;
			constexpr double grade = 0.894639;
			constexpr double road_incline_angle = 0.935285;
			constexpr double sine_road_incline_angle = 0.804768;
			constexpr double gravity = 9.80955;
			constexpr double gravity_times_sine_road_incline_angle = 7.89442;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.3774;
			constexpr double wind_direction = 0.128582;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 798.809;
			constexpr double air_temp = 44.1561;
			constexpr double pressure = 914.571;
			constexpr double air_density = 1.10883;
			constexpr double reciprocal_speed_of_sound = 0.00307337;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 12.8895;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.73115e+18;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {7.88334, -57.6837};
			constexpr GeographicalCoordinate end_coordinate = {53.8208, -129.562};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 5.21372;
			constexpr double weather_station = 0.304178;
			constexpr double distance = 87.4663;
			constexpr double heading = 4.98633;
			constexpr double elevation = -423.962;
			constexpr double grade = -0.936088;
			constexpr double road_incline_angle = -0.51343;
			constexpr double sine_road_incline_angle = -0.491168;
			constexpr double gravity = 9.78784;
			constexpr double gravity_times_sine_road_incline_angle = -4.80747;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 12.2685;
			constexpr double wind_direction = 3.3034;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 401.382;
			constexpr double air_temp = 15.2033;
			constexpr double pressure = 984.328;
			constexpr double air_density = 1.0612;
			constexpr double reciprocal_speed_of_sound = 0.00294158;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 5.89583;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.62441e+17;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 2") {
		constexpr double drag_coefficient = 0.00131068;
		constexpr double frontal_area = 8.72988;
		constexpr double array_area = 8.33835;
		constexpr double array_efficiency = 22.7766;
		constexpr double energy_capacity = 6027.51;
		constexpr double min_voltage = 88.8909;
		constexpr double max_voltage = 143.723;
		constexpr double resistance = 0.263758;
		constexpr double hysteresis_loss = 3.6611;
		constexpr double eddy_current_loss_coefficient = 0.0336272;
		constexpr double alpha = -5.93722;
		constexpr double beta = -7.69525;
		constexpr double a = 9.23052;
		constexpr double b = -5.62287e-06;
		constexpr double c = 0.587151;
		constexpr double pressure_at_stc = 159.023;
		constexpr double mass = 580.775;
		constexpr double wheel_radius = 0.404949;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {82.17, -21.7932};
			constexpr GeographicalCoordinate end_coordinate = {-50.1698, 57.7314};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 92.1306;
			constexpr double weather_station = 5.20145;
			constexpr double distance = 99.7472;
			constexpr double heading = 4.05201;
			constexpr double elevation = -485.854;
			constexpr double grade = -0.494746;
			constexpr double road_incline_angle = 0.272454;
			constexpr double sine_road_incline_angle = 0.269095;
			constexpr double gravity = 9.78809;
			constexpr double gravity_times_sine_road_incline_angle = 2.63393;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 49.8035;
			constexpr double wind_direction = 3.58695;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 4.98495;
			constexpr double air_temp = 9.10665;
			constexpr double pressure = 940.366;
			constexpr double air_density = 1.01012;
			constexpr double reciprocal_speed_of_sound = 0.00291438;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 27.377;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 42702.7;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-7.27797, 140.734};
			constexpr GeographicalCoordinate end_coordinate = {26.291, 74.4119};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 65.7411;
			constexpr double weather_station = 6.68092;
			constexpr double distance = 50.3628;
			constexpr double heading = 1.10843;
			constexpr double elevation = 495.998;
			constexpr double grade = -0.687052;
			constexpr double road_incline_angle = -0.20926;
			constexpr double sine_road_incline_angle = -0.207736;
			constexpr double gravity = 9.79641;
			constexpr double gravity_times_sine_road_incline_angle = -2.03507;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 28.0392;
			constexpr double wind_direction = 3.2219;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 997.727;
			constexpr double air_temp = 3.69275;
			constexpr double pressure = 946.052;
			constexpr double air_density = 1.26573;
			constexpr double reciprocal_speed_of_sound = 0.00293258;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 27.3969;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -32341.8;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-66.8144, 50.773};
			constexpr GeographicalCoordinate end_coordinate = {-28.7788, -42.7392};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 16.6802;
			constexpr double weather_station = 1.39199;
			constexpr double distance = 8.40425;
			constexpr double heading = 5.04045;
			constexpr double elevation = -91.8071;
			constexpr double grade = 0.523608;
			constexpr double road_incline_angle = 0.529471;
			constexpr double sine_road_incline_angle = 0.505077;
			constexpr double gravity = 9.81348;
			constexpr double gravity_times_sine_road_incline_angle = 4.95656;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 21.5524;
			constexpr double wind_direction = 4.06803;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 604.481;
			constexpr double air_temp = 13.1909;
			constexpr double pressure = 917.519;
			constexpr double air_density = 1.0734;
			constexpr double reciprocal_speed_of_sound = 0.00308049;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.0219;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 72247.4;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 3") {
		constexpr double drag_coefficient = 0.00490209;
		constexpr double frontal_area = 3.39217;
		constexpr double array_area = 5.64331;
		constexpr double array_efficiency = 28.4426;
		constexpr double energy_capacity = 6187.4;
		constexpr double min_voltage = 155.047;
		constexpr double max_voltage = 160.987;
		constexpr double resistance = 0.656365;
		constexpr double hysteresis_loss = 3.97445;
		constexpr double eddy_current_loss_coefficient = 0.0479923;
		constexpr double alpha = 4.17537;
		constexpr double beta = -2.15557;
		constexpr double a = -0.276843;
		constexpr double b = 6.92597e-06;
		constexpr double c = -0.0906901;
		constexpr double pressure_at_stc = 141.622;
		constexpr double mass = 861.648;
		constexpr double wheel_radius = 0.497305;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {36.7137, -29.4202};
			constexpr GeographicalCoordinate end_coordinate = {7.74658, 45.8321};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 90.1217;
			constexpr double weather_station = 4.01487;
			constexpr double distance = 42.7503;
			constexpr double heading = 3.62087;
			constexpr double elevation = 320.503;
			constexpr double grade = -0.64919;
			constexpr double road_incline_angle = 1.12525;
			constexpr double sine_road_incline_angle = 0.902375;
			constexpr double gravity = 9.79826;
			constexpr double gravity_times_sine_road_incline_angle = 8.8417;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 6.81523;
			constexpr double wind_direction = 1.77379;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 231.448;
			constexpr double air_temp = -0.2484;
			constexpr double pressure = 937.034;
			constexpr double air_density = 1.17353;
			constexpr double reciprocal_speed_of_sound = 0.00300977;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 21.5652;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.08066e+06;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-39.5207, -52.4015};
			constexpr GeographicalCoordinate end_coordinate = {-56.9429, -56.2272};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 124.005;
			constexpr double weather_station = 1.89058;
			constexpr double distance = 13.9706;
			constexpr double heading = 1.13094;
			constexpr double elevation = 70.1888;
			constexpr double grade = 0.768097;
			constexpr double road_incline_angle = -0.988444;
			constexpr double sine_road_incline_angle = -0.835171;
			constexpr double gravity = 9.79705;
			constexpr double gravity_times_sine_road_incline_angle = -8.18222;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 0.253864;
			constexpr double wind_direction = 3.91956;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 573.496;
			constexpr double air_temp = -27.0924;
			constexpr double pressure = 967.366;
			constexpr double air_density = 1.04597;
			constexpr double reciprocal_speed_of_sound = 0.00303742;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 23.1805;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.70991e+06;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-75.2958, -167.587};
			constexpr GeographicalCoordinate end_coordinate = {38.1367, 123.092};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 159.056;
			constexpr double weather_station = 1.59963;
			constexpr double distance = 86.1681;
			constexpr double heading = 1.10415;
			constexpr double elevation = 123.055;
			constexpr double grade = -0.0637868;
			constexpr double road_incline_angle = 0.190983;
			constexpr double sine_road_incline_angle = 0.189824;
			constexpr double gravity = 9.78742;
			constexpr double gravity_times_sine_road_incline_angle = 1.85789;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 18.8077;
			constexpr double wind_direction = 0.443056;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 380.783;
			constexpr double air_temp = 23.8906;
			constexpr double pressure = 1034.82;
			constexpr double air_density = 1.14641;
			constexpr double reciprocal_speed_of_sound = 0.00293891;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 13.8158;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -306167;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 4") {
		constexpr double drag_coefficient = 0.00371557;
		constexpr double frontal_area = 5.29219;
		constexpr double array_area = 5.69445;
		constexpr double array_efficiency = 22.4478;
		constexpr double energy_capacity = 3470.44;
		constexpr double min_voltage = 105.815;
		constexpr double max_voltage = 128.751;
		constexpr double resistance = 0.753034;
		constexpr double hysteresis_loss = 4.79474;
		constexpr double eddy_current_loss_coefficient = 0.00715481;
		constexpr double alpha = -2.845;
		constexpr double beta = 7.50969;
		constexpr double a = 0.066638;
		constexpr double b = -3.15269e-07;
		constexpr double c = -0.0164319;
		constexpr double pressure_at_stc = 143.018;
		constexpr double mass = 581.65;
		constexpr double wheel_radius = 0.293971;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {89.4045, 14.5121};
			constexpr GeographicalCoordinate end_coordinate = {30.5251, 50.0705};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 18.373;
			constexpr double weather_station = 3.40046;
			constexpr double distance = 91.8514;
			constexpr double heading = 3.1643;
			constexpr double elevation = 18.6689;
			constexpr double grade = -0.280423;
			constexpr double road_incline_angle = 1.55437;
			constexpr double sine_road_incline_angle = 0.999865;
			constexpr double gravity = 9.78043;
			constexpr double gravity_times_sine_road_incline_angle = 9.77911;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 38.4746;
			constexpr double wind_direction = 4.5632;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 218.927;
			constexpr double air_temp = 47.4377;
			constexpr double pressure = 929.581;
			constexpr double air_density = 1.1154;
			constexpr double reciprocal_speed_of_sound = 0.00292863;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 8.65932;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.25882e+21;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {33.3585, -130.283};
			constexpr GeographicalCoordinate end_coordinate = {60.5036, 149.534};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 80.3703;
			constexpr double weather_station = 1.23407;
			constexpr double distance = 90.9134;
			constexpr double heading = 2.97905;
			constexpr double elevation = 460.358;
			constexpr double grade = 0.132454;
			constexpr double road_incline_angle = -1.55905;
			constexpr double sine_road_incline_angle = -0.999931;
			constexpr double gravity = 9.80904;
			constexpr double gravity_times_sine_road_incline_angle = -9.80836;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 23.1738;
			constexpr double wind_direction = 1.27316;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 163.665;
			constexpr double air_temp = 14.0055;
			constexpr double pressure = 1075.69;
			constexpr double air_density = 1.12099;
			constexpr double reciprocal_speed_of_sound = 0.0029942;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 16.0203;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -8.17222e+21;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {52.3038, 97.9462};
			constexpr GeographicalCoordinate end_coordinate = {51.5541, 6.01198};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 80.2332;
			constexpr double weather_station = 3.03061;
			constexpr double distance = 78.6046;
			constexpr double heading = 3.40579;
			constexpr double elevation = 328.28;
			constexpr double grade = 0.99816;
			constexpr double road_incline_angle = 0.229852;
			constexpr double sine_road_incline_angle = 0.227833;
			constexpr double gravity = 9.81017;
			constexpr double gravity_times_sine_road_incline_angle = 2.23508;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 19.6522;
			constexpr double wind_direction = 5.16387;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 742.008;
			constexpr double air_temp = 12.5658;
			constexpr double pressure = 1025.05;
			constexpr double air_density = 1.28916;
			constexpr double reciprocal_speed_of_sound = 0.00306605;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 29.7168;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -5.225e+22;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 5") {
		constexpr double drag_coefficient = 0.00989177;
		constexpr double frontal_area = 7.54004;
		constexpr double array_area = 3.88236;
		constexpr double array_efficiency = 23.5293;
		constexpr double energy_capacity = 4395.25;
		constexpr double min_voltage = 145.577;
		constexpr double max_voltage = 150.916;
		constexpr double resistance = 0.496602;
		constexpr double hysteresis_loss = 4.0959;
		constexpr double eddy_current_loss_coefficient = 0.0179007;
		constexpr double alpha = -6.04415;
		constexpr double beta = -2.49581;
		constexpr double a = -5.22421;
		constexpr double b = -7.98562e-06;
		constexpr double c = 0.23313;
		constexpr double pressure_at_stc = 181.13;
		constexpr double mass = 119.792;
		constexpr double wheel_radius = 0.208999;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {88.7591, 19.773};
			constexpr GeographicalCoordinate end_coordinate = {-38.5879, 80.0144};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 69.7174;
			constexpr double weather_station = 3.69203;
			constexpr double distance = 48.7719;
			constexpr double heading = 1.56283;
			constexpr double elevation = 195.408;
			constexpr double grade = 0.930048;
			constexpr double road_incline_angle = 0.15084;
			constexpr double sine_road_incline_angle = 0.150268;
			constexpr double gravity = 9.80243;
			constexpr double gravity_times_sine_road_incline_angle = 1.47299;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 49.6004;
			constexpr double wind_direction = 3.67364;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 405.21;
			constexpr double air_temp = -31.7044;
			constexpr double pressure = 927.099;
			constexpr double air_density = 1.27596;
			constexpr double reciprocal_speed_of_sound = 0.00290044;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 22.5899;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 4001.19;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-46.3943, 15.4039};
			constexpr GeographicalCoordinate end_coordinate = {32.4166, 48.9291};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 17.8803;
			constexpr double weather_station = 6.84086;
			constexpr double distance = 53.0233;
			constexpr double heading = 6.14139;
			constexpr double elevation = 195.617;
			constexpr double grade = -0.635815;
			constexpr double road_incline_angle = 0.614925;
			constexpr double sine_road_incline_angle = 0.576897;
			constexpr double gravity = 9.78798;
			constexpr double gravity_times_sine_road_incline_angle = 5.64666;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 46.9716;
			constexpr double wind_direction = 3.0881;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 429.616;
			constexpr double air_temp = -45.6373;
			constexpr double pressure = 1076.44;
			constexpr double air_density = 1.09592;
			constexpr double reciprocal_speed_of_sound = 0.00298902;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 26.3027;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 18249.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-10.4153, -136.078};
			constexpr GeographicalCoordinate end_coordinate = {58.779, 154.48};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 180.806;
			constexpr double weather_station = 0.736614;
			constexpr double distance = 50.7792;
			constexpr double heading = 4.22872;
			constexpr double elevation = 457.482;
			constexpr double grade = -0.65924;
			constexpr double road_incline_angle = -0.45786;
			constexpr double sine_road_incline_angle = -0.44203;
			constexpr double gravity = 9.79789;
			constexpr double gravity_times_sine_road_incline_angle = -4.33096;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 10.4515;
			constexpr double wind_direction = 2.87962;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 400.628;
			constexpr double air_temp = 34.1143;
			constexpr double pressure = 938.039;
			constexpr double air_density = 1.25724;
			constexpr double reciprocal_speed_of_sound = 0.0029588;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 25.6965;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -12381.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 6") {
		constexpr double drag_coefficient = 0.000889964;
		constexpr double frontal_area = 2.02018;
		constexpr double array_area = 2.14896;
		constexpr double array_efficiency = 17.2303;
		constexpr double energy_capacity = 2686.61;
		constexpr double min_voltage = 106.156;
		constexpr double max_voltage = 145.114;
		constexpr double resistance = 0.938976;
		constexpr double hysteresis_loss = 4.75638;
		constexpr double eddy_current_loss_coefficient = 0.0206062;
		constexpr double alpha = -6.83522;
		constexpr double beta = -0.959055;
		constexpr double a = 5.34655;
		constexpr double b = 6.33531e-06;
		constexpr double c = -0.883873;
		constexpr double pressure_at_stc = 120.963;
		constexpr double mass = 830.489;
		constexpr double wheel_radius = 0.215403;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-18.3914, 148.426};
			constexpr GeographicalCoordinate end_coordinate = {17.5197, 32.1356};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 89.812;
			constexpr double weather_station = 1.30959;
			constexpr double distance = 15.8392;
			constexpr double heading = 4.63763;
			constexpr double elevation = -314.35;
			constexpr double grade = 0.491434;
			constexpr double road_incline_angle = -0.416059;
			constexpr double sine_road_incline_angle = -0.404159;
			constexpr double gravity = 9.78207;
			constexpr double gravity_times_sine_road_incline_angle = -3.95351;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 49.1079;
			constexpr double wind_direction = 3.77814;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 952.169;
			constexpr double air_temp = 47.1438;
			constexpr double pressure = 1045.68;
			constexpr double air_density = 1.09793;
			constexpr double reciprocal_speed_of_sound = 0.00294082;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 3.30994;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -10858.5;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {55.5858, 117.113};
			constexpr GeographicalCoordinate end_coordinate = {-49.895, -124.765};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 16.1347;
			constexpr double weather_station = 3.53483;
			constexpr double distance = 18.5047;
			constexpr double heading = 1.92414;
			constexpr double elevation = -421.313;
			constexpr double grade = -0.906006;
			constexpr double road_incline_angle = -0.411085;
			constexpr double sine_road_incline_angle = -0.399604;
			constexpr double gravity = 9.79627;
			constexpr double gravity_times_sine_road_incline_angle = -3.91463;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 25.8159;
			constexpr double wind_direction = 3.96219;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 7.46226;
			constexpr double air_temp = -38.7232;
			constexpr double pressure = 956.795;
			constexpr double air_density = 1.22334;
			constexpr double reciprocal_speed_of_sound = 0.00297529;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 27.4382;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -89188.3;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {34.1371, 34.1275};
			constexpr GeographicalCoordinate end_coordinate = {-39.3417, 57.476};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 159.85;
			constexpr double weather_station = 9.38151;
			constexpr double distance = 18.7803;
			constexpr double heading = 5.63627;
			constexpr double elevation = 472.098;
			constexpr double grade = 0.0405319;
			constexpr double road_incline_angle = 0.806554;
			constexpr double sine_road_incline_angle = 0.721907;
			constexpr double gravity = 9.78562;
			constexpr double gravity_times_sine_road_incline_angle = 7.06431;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 34.1486;
			constexpr double wind_direction = 2.33885;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 401.863;
			constexpr double air_temp = -30.1333;
			constexpr double pressure = 1083.07;
			constexpr double air_density = 1.11317;
			constexpr double reciprocal_speed_of_sound = 0.00303347;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 17.7927;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 104398;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 7") {
		constexpr double drag_coefficient = 0.000785233;
		constexpr double frontal_area = 7.30694;
		constexpr double array_area = 8.33168;
		constexpr double array_efficiency = 16.3279;
		constexpr double energy_capacity = 3371.85;
		constexpr double min_voltage = 130.198;
		constexpr double max_voltage = 154.085;
		constexpr double resistance = 0.402095;
		constexpr double hysteresis_loss = 2.47233;
		constexpr double eddy_current_loss_coefficient = 0.0492111;
		constexpr double alpha = -3.13609;
		constexpr double beta = 4.68016;
		constexpr double a = 9.40886;
		constexpr double b = -7.30967e-06;
		constexpr double c = -0.588583;
		constexpr double pressure_at_stc = 176.898;
		constexpr double mass = 822.875;
		constexpr double wheel_radius = 0.154292;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {85.5324, 161.248};
			constexpr GeographicalCoordinate end_coordinate = {-1.96362, 12.8249};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 67.329;
			constexpr double weather_station = 5.17829;
			constexpr double distance = 55.6511;
			constexpr double heading = 3.04096;
			constexpr double elevation = -336.863;
			constexpr double grade = -0.823597;
			constexpr double road_incline_angle = 0.479116;
			constexpr double sine_road_incline_angle = 0.460995;
			constexpr double gravity = 9.78146;
			constexpr double gravity_times_sine_road_incline_angle = 4.5092;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 48.6247;
			constexpr double wind_direction = 2.12049;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 308.344;
			constexpr double air_temp = 19.4762;
			constexpr double pressure = 910.238;
			constexpr double air_density = 1.21762;
			constexpr double reciprocal_speed_of_sound = 0.00299327;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 28.3362;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -5.16777e+14;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-82.8916, -82.0523};
			constexpr GeographicalCoordinate end_coordinate = {60.0136, -23.4853};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 147.68;
			constexpr double weather_station = 8.4112;
			constexpr double distance = 86.7451;
			constexpr double heading = 4.94202;
			constexpr double elevation = 180.543;
			constexpr double grade = -0.549669;
			constexpr double road_incline_angle = 0.259616;
			constexpr double sine_road_incline_angle = 0.25671;
			constexpr double gravity = 9.79308;
			constexpr double gravity_times_sine_road_incline_angle = 2.51398;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 34.9936;
			constexpr double wind_direction = 5.55977;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 754.403;
			constexpr double air_temp = 25.7992;
			constexpr double pressure = 923.985;
			constexpr double air_density = 1.22236;
			constexpr double reciprocal_speed_of_sound = 0.00297322;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 13.6514;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -5.7811e+13;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {79.3964, 9.13663};
			constexpr GeographicalCoordinate end_coordinate = {31.7513, -104.019};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 189.296;
			constexpr double weather_station = 9.46728;
			constexpr double distance = 17.285;
			constexpr double heading = 1.21784;
			constexpr double elevation = 234.61;
			constexpr double grade = 0.566588;
			constexpr double road_incline_angle = 0.643689;
			constexpr double sine_road_incline_angle = 0.60015;
			constexpr double gravity = 9.79104;
			constexpr double gravity_times_sine_road_incline_angle = 5.8761;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 3.25987;
			constexpr double wind_direction = 0.841679;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 906.374;
			constexpr double air_temp = 32.6433;
			constexpr double pressure = 1028.1;
			constexpr double air_density = 1.06772;
			constexpr double reciprocal_speed_of_sound = 0.00295094;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 8.5283;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -1.39347e+13;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 8") {
		constexpr double drag_coefficient = 0.00714015;
		constexpr double frontal_area = 7.52495;
		constexpr double array_area = 4.67337;
		constexpr double array_efficiency = 23.4711;
		constexpr double energy_capacity = 378.909;
		constexpr double min_voltage = 80.2354;
		constexpr double max_voltage = 130.212;
		constexpr double resistance = 0.636863;
		constexpr double hysteresis_loss = 2.86172;
		constexpr double eddy_current_loss_coefficient = 0.00185779;
		constexpr double alpha = -9.33695;
		constexpr double beta = 0.835492;
		constexpr double a = -7.99366;
		constexpr double b = 8.15885e-06;
		constexpr double c = -0.0250766;
		constexpr double pressure_at_stc = 177.548;
		constexpr double mass = 586.885;
		constexpr double wheel_radius = 0.316562;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {17.8851, 26.877};
			constexpr GeographicalCoordinate end_coordinate = {64.2236, 13.8431};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 27.8453;
			constexpr double weather_station = 2.48932;
			constexpr double distance = 69.0333;
			constexpr double heading = 4.21518;
			constexpr double elevation = -308.48;
			constexpr double grade = -0.578216;
			constexpr double road_incline_angle = -1.37338;
			constexpr double sine_road_incline_angle = -0.980577;
			constexpr double gravity = 9.80231;
			constexpr double gravity_times_sine_road_incline_angle = -9.61192;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 44.8819;
			constexpr double wind_direction = 5.2381;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 598.345;
			constexpr double air_temp = 39.0647;
			constexpr double pressure = 1000.97;
			constexpr double air_density = 1.05879;
			constexpr double reciprocal_speed_of_sound = 0.00295229;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 8.83267;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -49562.3;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-28.4876, 52.596};
			constexpr GeographicalCoordinate end_coordinate = {63.3013, 67.6592};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 141.97;
			constexpr double weather_station = 3.11745;
			constexpr double distance = 30.8503;
			constexpr double heading = 4.5101;
			constexpr double elevation = -256.741;
			constexpr double grade = 0.0303802;
			constexpr double road_incline_angle = 0.180672;
			constexpr double sine_road_incline_angle = 0.179691;
			constexpr double gravity = 9.7843;
			constexpr double gravity_times_sine_road_incline_angle = 1.75815;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 34.7847;
			constexpr double wind_direction = 5.72353;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 906.441;
			constexpr double air_temp = -10.7977;
			constexpr double pressure = 1041.2;
			constexpr double air_density = 1.2695;
			constexpr double reciprocal_speed_of_sound = 0.00290846;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 22.4058;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 24035.5;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-53.6263, 119.235};
			constexpr GeographicalCoordinate end_coordinate = {70.7604, -137.024};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 10.4275;
			constexpr double weather_station = 3.89264;
			constexpr double distance = 11.5967;
			constexpr double heading = 1.8985;
			constexpr double elevation = 104.846;
			constexpr double grade = -0.383228;
			constexpr double road_incline_angle = -1.10178;
			constexpr double sine_road_incline_angle = -0.892016;
			constexpr double gravity = 9.80781;
			constexpr double gravity_times_sine_road_incline_angle = -8.74871;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 5.07776;
			constexpr double wind_direction = 1.22119;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 818.809;
			constexpr double air_temp = -49.3915;
			constexpr double pressure = 973.881;
			constexpr double air_density = 1.13197;
			constexpr double reciprocal_speed_of_sound = 0.0029867;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 5.37743;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -27593.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 9") {
		constexpr double drag_coefficient = 0.00335103;
		constexpr double frontal_area = 1.54604;
		constexpr double array_area = 6.39688;
		constexpr double array_efficiency = 28.3684;
		constexpr double energy_capacity = 4366.82;
		constexpr double min_voltage = 94.2053;
		constexpr double max_voltage = 130.346;
		constexpr double resistance = 0.0903753;
		constexpr double hysteresis_loss = 3.27636;
		constexpr double eddy_current_loss_coefficient = 0.0382279;
		constexpr double alpha = 2.93103;
		constexpr double beta = -6.98544;
		constexpr double a = -4.93134;
		constexpr double b = 7.26864e-06;
		constexpr double c = 0.278811;
		constexpr double pressure_at_stc = 141.267;
		constexpr double mass = 406.913;
		constexpr double wheel_radius = 0.30808;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-12.1628, 136.552};
			constexpr GeographicalCoordinate end_coordinate = {1.07715, -94.9922};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 173.771;
			constexpr double weather_station = 2.36896;
			constexpr double distance = 14.9312;
			constexpr double heading = 5.67663;
			constexpr double elevation = 293.959;
			constexpr double grade = 0.647971;
			constexpr double road_incline_angle = -0.114208;
			constexpr double sine_road_incline_angle = -0.11396;
			constexpr double gravity = 9.813;
			constexpr double gravity_times_sine_road_incline_angle = -1.11829;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 30.4959;
			constexpr double wind_direction = 4.27513;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 362.002;
			constexpr double air_temp = -0.0937407;
			constexpr double pressure = 1035.27;
			constexpr double air_density = 1.28864;
			constexpr double reciprocal_speed_of_sound = 0.00291242;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 14.1129;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -6399.57;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {42.9806, -162.741};
			constexpr GeographicalCoordinate end_coordinate = {-14.7959, -40.8742};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 70.7404;
			constexpr double weather_station = 0.409187;
			constexpr double distance = 64.0503;
			constexpr double heading = 5.95986;
			constexpr double elevation = 122.097;
			constexpr double grade = -0.0885708;
			constexpr double road_incline_angle = -1.27101;
			constexpr double sine_road_incline_angle = -0.955398;
			constexpr double gravity = 9.7825;
			constexpr double gravity_times_sine_road_incline_angle = -9.34619;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 15.9349;
			constexpr double wind_direction = 4.13778;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 820.838;
			constexpr double air_temp = -28.6857;
			constexpr double pressure = 1090.34;
			constexpr double air_density = 1.09065;
			constexpr double reciprocal_speed_of_sound = 0.00309;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 9.36644;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = -35616.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-19.3341, 13.3239};
			constexpr GeographicalCoordinate end_coordinate = {54.717, -148.549};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 109.872;
			constexpr double weather_station = 0.380263;
			constexpr double distance = 29.3682;
			constexpr double heading = 2.1925;
			constexpr double elevation = -139.411;
			constexpr double grade = 0.374693;
			constexpr double road_incline_angle = 0.749432;
			constexpr double sine_road_incline_angle = 0.681223;
			constexpr double gravity = 9.81727;
			constexpr double gravity_times_sine_road_incline_angle = 6.68775;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 5.39676;
			constexpr double wind_direction = 0.123596;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 389.071;
			constexpr double air_temp = 38.176;
			constexpr double pressure = 1014.93;
			constexpr double air_density = 1.279;
			constexpr double reciprocal_speed_of_sound = 0.00302802;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double speed = 22.8677;

			const double result = runner.calculate_power_out(route_segment, weather_data, speed);
			constexpr double expected = 62267.8;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
}
TEST_CASE("RaceSegmentRunner: calculate_power_in", "[RaceSegmentRunner]") {
	SECTION("Random Test 0") {
		constexpr double drag_coefficient = 0.00538748;
		constexpr double frontal_area = 1.85474;
		constexpr double array_area = 3.66005;
		constexpr double array_efficiency = 18.5874;
		constexpr double energy_capacity = 7408.2;
		constexpr double min_voltage = 110.551;
		constexpr double max_voltage = 136.499;
		constexpr double resistance = 0.679414;
		constexpr double hysteresis_loss = 2.73538;
		constexpr double eddy_current_loss_coefficient = 0.0132067;
		constexpr double alpha = -7.81105;
		constexpr double beta = 5.41092;
		constexpr double a = 2.86654;
		constexpr double b = 1.28896e-07;
		constexpr double c = -0.176997;
		constexpr double pressure_at_stc = 177.216;
		constexpr double mass = 111.697;
		constexpr double wheel_radius = 0.217283;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-47.5881, -95.6402};
			constexpr GeographicalCoordinate end_coordinate = {3.79137, -94.0127};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 25.9074;
			constexpr double weather_station = 9.99544;
			constexpr double distance = 23.107;
			constexpr double heading = 1.03489;
			constexpr double elevation = -159.183;
			constexpr double grade = -0.43939;
			constexpr double road_incline_angle = -0.33905;
			constexpr double sine_road_incline_angle = -0.332591;
			constexpr double gravity = 9.79271;
			constexpr double gravity_times_sine_road_incline_angle = -3.25697;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 13.9486;
			constexpr double wind_direction = 2.61;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 415.785;
			constexpr double air_temp = -14.323;
			constexpr double pressure = 947.988;
			constexpr double air_density = 1.13013;
			constexpr double reciprocal_speed_of_sound = 0.00300529;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 282.861;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {64.8657, 57.5277};
			constexpr GeographicalCoordinate end_coordinate = {26.8906, -115.39};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 194.029;
			constexpr double weather_station = 1.94605;
			constexpr double distance = 33.693;
			constexpr double heading = 1.94842;
			constexpr double elevation = -309.786;
			constexpr double grade = 0.518401;
			constexpr double road_incline_angle = -1.37187;
			constexpr double sine_road_incline_angle = -0.980279;
			constexpr double gravity = 9.80065;
			constexpr double gravity_times_sine_road_incline_angle = -9.60737;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 20.7906;
			constexpr double wind_direction = 3.49181;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 775.308;
			constexpr double air_temp = 1.44202;
			constexpr double pressure = 1082.88;
			constexpr double air_density = 1.14343;
			constexpr double reciprocal_speed_of_sound = 0.00308322;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 527.447;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {4.16742, -43.4894};
			constexpr GeographicalCoordinate end_coordinate = {63.2579, 119.824};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 64.755;
			constexpr double weather_station = 7.6351;
			constexpr double distance = 82.6594;
			constexpr double heading = 4.45034;
			constexpr double elevation = 456.089;
			constexpr double grade = 0.512798;
			constexpr double road_incline_angle = 0.1092;
			constexpr double sine_road_incline_angle = 0.108983;
			constexpr double gravity = 9.80526;
			constexpr double gravity_times_sine_road_incline_angle = 1.06861;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 9.13155;
			constexpr double wind_direction = 5.0586;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 268.106;
			constexpr double air_temp = 43.5363;
			constexpr double pressure = 1062.85;
			constexpr double air_density = 1.2724;
			constexpr double reciprocal_speed_of_sound = 0.00305691;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 182.394;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 1") {
		constexpr double drag_coefficient = 0.00665466;
		constexpr double frontal_area = 5.09452;
		constexpr double array_area = 8.2909;
		constexpr double array_efficiency = 29.802;
		constexpr double energy_capacity = 6549.42;
		constexpr double min_voltage = 144.195;
		constexpr double max_voltage = 145.099;
		constexpr double resistance = 0.249903;
		constexpr double hysteresis_loss = 3.39249;
		constexpr double eddy_current_loss_coefficient = 0.031261;
		constexpr double alpha = 4.85875;
		constexpr double beta = -1.14462;
		constexpr double a = -1.51066;
		constexpr double b = -2.64381e-06;
		constexpr double c = 0.581579;
		constexpr double pressure_at_stc = 158.391;
		constexpr double mass = 194.513;
		constexpr double wheel_radius = 0.10969;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-25.3561, 9.26261};
			constexpr GeographicalCoordinate end_coordinate = {11.1523, -176.219};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 4.72934;
			constexpr double weather_station = 7.7943;
			constexpr double distance = 95.9673;
			constexpr double heading = 5.45043;
			constexpr double elevation = 330.35;
			constexpr double grade = 0.513811;
			constexpr double road_incline_angle = -1.00248;
			constexpr double sine_road_incline_angle = -0.842811;
			constexpr double gravity = 9.78215;
			constexpr double gravity_times_sine_road_incline_angle = -8.24451;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 13.106;
			constexpr double wind_direction = 0.177075;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 677.348;
			constexpr double air_temp = 0.600624;
			constexpr double pressure = 957.292;
			constexpr double air_density = 1.1051;
			constexpr double reciprocal_speed_of_sound = 0.00303661;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1673.63;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-10.9344, -14.2974};
			constexpr GeographicalCoordinate end_coordinate = {-43.3207, 125.423};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 167.034;
			constexpr double weather_station = 5.3345;
			constexpr double distance = 24.4477;
			constexpr double heading = 3.80383;
			constexpr double elevation = 358.178;
			constexpr double grade = 0.279607;
			constexpr double road_incline_angle = 0.341292;
			constexpr double sine_road_incline_angle = 0.334705;
			constexpr double gravity = 9.80562;
			constexpr double gravity_times_sine_road_incline_angle = 3.28199;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.6444;
			constexpr double wind_direction = 4.99028;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 415.46;
			constexpr double air_temp = -39.7009;
			constexpr double pressure = 1080.11;
			constexpr double air_density = 1.22038;
			constexpr double reciprocal_speed_of_sound = 0.00306304;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1026.54;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-75.3983, -175.343};
			constexpr GeographicalCoordinate end_coordinate = {-66.2101, -169.275};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 4.03813;
			constexpr double weather_station = 5.56528;
			constexpr double distance = 29.4183;
			constexpr double heading = 3.39823;
			constexpr double elevation = -374.181;
			constexpr double grade = 0.34535;
			constexpr double road_incline_angle = -0.559406;
			constexpr double sine_road_incline_angle = -0.530683;
			constexpr double gravity = 9.79755;
			constexpr double gravity_times_sine_road_incline_angle = -5.1994;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 31.4599;
			constexpr double wind_direction = 6.05568;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 87.1021;
			constexpr double air_temp = -3.74303;
			constexpr double pressure = 997.018;
			constexpr double air_density = 1.259;
			constexpr double reciprocal_speed_of_sound = 0.00295412;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 215.217;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 2") {
		constexpr double drag_coefficient = 0.00160881;
		constexpr double frontal_area = 7.93154;
		constexpr double array_area = 3.7206;
		constexpr double array_efficiency = 17.7305;
		constexpr double energy_capacity = 5829.13;
		constexpr double min_voltage = 89.9229;
		constexpr double max_voltage = 142.336;
		constexpr double resistance = 0.400092;
		constexpr double hysteresis_loss = 3.21973;
		constexpr double eddy_current_loss_coefficient = 0.0358038;
		constexpr double alpha = -9.03919;
		constexpr double beta = 0.285515;
		constexpr double a = -6.22219;
		constexpr double b = -3.6694e-06;
		constexpr double c = 0.206744;
		constexpr double pressure_at_stc = 192.266;
		constexpr double mass = 315.518;
		constexpr double wheel_radius = 0.117737;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {61.5446, 12.1455};
			constexpr GeographicalCoordinate end_coordinate = {-0.85834, -106.37};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 177.664;
			constexpr double weather_station = 8.49895;
			constexpr double distance = 71.9107;
			constexpr double heading = 3.14484;
			constexpr double elevation = 120.982;
			constexpr double grade = 0.243351;
			constexpr double road_incline_angle = -1.52923;
			constexpr double sine_road_incline_angle = -0.999136;
			constexpr double gravity = 9.8182;
			constexpr double gravity_times_sine_road_incline_angle = -9.80972;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 18.2453;
			constexpr double wind_direction = 1.77385;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 955.9;
			constexpr double air_temp = 17.773;
			constexpr double pressure = 1054.51;
			constexpr double air_density = 1.25918;
			constexpr double reciprocal_speed_of_sound = 0.00292168;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 630.588;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {54.55, -21.776};
			constexpr GeographicalCoordinate end_coordinate = {3.33394, -66.66};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 166.827;
			constexpr double weather_station = 3.81886;
			constexpr double distance = 83.2217;
			constexpr double heading = 3.51882;
			constexpr double elevation = 351.594;
			constexpr double grade = -0.685211;
			constexpr double road_incline_angle = 0.077445;
			constexpr double sine_road_incline_angle = 0.0773676;
			constexpr double gravity = 9.79146;
			constexpr double gravity_times_sine_road_incline_angle = 0.757542;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 22.9299;
			constexpr double wind_direction = 1.81404;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 247.094;
			constexpr double air_temp = 25.7728;
			constexpr double pressure = 996.775;
			constexpr double air_density = 1.126;
			constexpr double reciprocal_speed_of_sound = 0.00298013;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 163.003;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-73.4334, -102.027};
			constexpr GeographicalCoordinate end_coordinate = {-84.1648, -103.058};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 73.4971;
			constexpr double weather_station = 5.37442;
			constexpr double distance = 16.2956;
			constexpr double heading = 4.57229;
			constexpr double elevation = 352.98;
			constexpr double grade = 0.634641;
			constexpr double road_incline_angle = -0.0693265;
			constexpr double sine_road_incline_angle = -0.069271;
			constexpr double gravity = 9.8047;
			constexpr double gravity_times_sine_road_incline_angle = -0.679181;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.5538;
			constexpr double wind_direction = 2.34483;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 114.087;
			constexpr double air_temp = -5.67246;
			constexpr double pressure = 1041.82;
			constexpr double air_density = 1.12498;
			constexpr double reciprocal_speed_of_sound = 0.00303784;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 75.2609;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 3") {
		constexpr double drag_coefficient = 0.00126415;
		constexpr double frontal_area = 3.33917;
		constexpr double array_area = 6.62868;
		constexpr double array_efficiency = 24.369;
		constexpr double energy_capacity = 5344.5;
		constexpr double min_voltage = 114.111;
		constexpr double max_voltage = 121.393;
		constexpr double resistance = 0.34116;
		constexpr double hysteresis_loss = 2.51945;
		constexpr double eddy_current_loss_coefficient = 0.023637;
		constexpr double alpha = 5.8772;
		constexpr double beta = -0.00687775;
		constexpr double a = 3.52641;
		constexpr double b = -6.74915e-06;
		constexpr double c = 0.261434;
		constexpr double pressure_at_stc = 143.885;
		constexpr double mass = 825.452;
		constexpr double wheel_radius = 0.274397;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-20.2015, 122.6};
			constexpr GeographicalCoordinate end_coordinate = {-61.7776, 31.3409};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 34.1694;
			constexpr double weather_station = 6.74318;
			constexpr double distance = 1.8636;
			constexpr double heading = 5.57643;
			constexpr double elevation = 128.636;
			constexpr double grade = 0.876597;
			constexpr double road_incline_angle = -1.42349;
			constexpr double sine_road_incline_angle = -0.98917;
			constexpr double gravity = 9.78125;
			constexpr double gravity_times_sine_road_incline_angle = -9.67532;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 34.0677;
			constexpr double wind_direction = 0.517965;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 321.051;
			constexpr double air_temp = -44.5575;
			constexpr double pressure = 1011.33;
			constexpr double air_density = 1.08635;
			constexpr double reciprocal_speed_of_sound = 0.00295486;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 518.608;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-26.967, -76.9738};
			constexpr GeographicalCoordinate end_coordinate = {38.4834, 130.19};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 130.677;
			constexpr double weather_station = 6.50817;
			constexpr double distance = 31.6688;
			constexpr double heading = 3.32755;
			constexpr double elevation = -181.564;
			constexpr double grade = -0.252211;
			constexpr double road_incline_angle = -0.0241561;
			constexpr double sine_road_incline_angle = -0.0241537;
			constexpr double gravity = 9.78809;
			constexpr double gravity_times_sine_road_incline_angle = -0.236419;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 37.9207;
			constexpr double wind_direction = 4.54535;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 715.751;
			constexpr double air_temp = -0.964678;
			constexpr double pressure = 1057.02;
			constexpr double air_density = 1.20592;
			constexpr double reciprocal_speed_of_sound = 0.00296927;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1156.18;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {62.9051, 81.7292};
			constexpr GeographicalCoordinate end_coordinate = {12.4508, -153.606};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 81.3901;
			constexpr double weather_station = 7.1933;
			constexpr double distance = 96.3319;
			constexpr double heading = 0.589943;
			constexpr double elevation = -211.168;
			constexpr double grade = 0.216434;
			constexpr double road_incline_angle = 0.407533;
			constexpr double sine_road_incline_angle = 0.396346;
			constexpr double gravity = 9.78516;
			constexpr double gravity_times_sine_road_incline_angle = 3.87831;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 44.8418;
			constexpr double wind_direction = 5.37315;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 858.767;
			constexpr double air_temp = 31.3504;
			constexpr double pressure = 1046.06;
			constexpr double air_density = 1.15848;
			constexpr double reciprocal_speed_of_sound = 0.0029616;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1387.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 4") {
		constexpr double drag_coefficient = 0.00847755;
		constexpr double frontal_area = 5.384;
		constexpr double array_area = 2.44174;
		constexpr double array_efficiency = 15.908;
		constexpr double energy_capacity = 1019.07;
		constexpr double min_voltage = 87.1831;
		constexpr double max_voltage = 122.67;
		constexpr double resistance = 0.208956;
		constexpr double hysteresis_loss = 1.5745;
		constexpr double eddy_current_loss_coefficient = 0.0329679;
		constexpr double alpha = 4.88875;
		constexpr double beta = -3.84157;
		constexpr double a = 1.45702;
		constexpr double b = -2.99901e-06;
		constexpr double c = 0.57581;
		constexpr double pressure_at_stc = 115.734;
		constexpr double mass = 957.433;
		constexpr double wheel_radius = 0.39086;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {35.3608, 179.984};
			constexpr GeographicalCoordinate end_coordinate = {-72.5598, -34.7424};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 165.52;
			constexpr double weather_station = 6.88904;
			constexpr double distance = 14.4174;
			constexpr double heading = 5.45116;
			constexpr double elevation = -275.35;
			constexpr double grade = -0.898679;
			constexpr double road_incline_angle = -0.249724;
			constexpr double sine_road_incline_angle = -0.247136;
			constexpr double gravity = 9.7857;
			constexpr double gravity_times_sine_road_incline_angle = -2.4184;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 26.9688;
			constexpr double wind_direction = 0.901449;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 7.31746;
			constexpr double air_temp = -44.9355;
			constexpr double pressure = 1088.41;
			constexpr double air_density = 1.13367;
			constexpr double reciprocal_speed_of_sound = 0.00292286;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 2.84233;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {84.6996, 115.123};
			constexpr GeographicalCoordinate end_coordinate = {55.2033, 140.435};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 19.0761;
			constexpr double weather_station = 3.46115;
			constexpr double distance = 73.0701;
			constexpr double heading = 1.99025;
			constexpr double elevation = -316.284;
			constexpr double grade = -0.605413;
			constexpr double road_incline_angle = -0.377391;
			constexpr double sine_road_incline_angle = -0.368496;
			constexpr double gravity = 9.81884;
			constexpr double gravity_times_sine_road_incline_angle = -3.6182;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 17.9731;
			constexpr double wind_direction = 2.67615;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 452.556;
			constexpr double air_temp = -40.3757;
			constexpr double pressure = 1091.36;
			constexpr double air_density = 1.16244;
			constexpr double reciprocal_speed_of_sound = 0.00298913;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 175.787;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-33.9913, -53.5774};
			constexpr GeographicalCoordinate end_coordinate = {59.3311, 105.759};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 194.193;
			constexpr double weather_station = 0.280222;
			constexpr double distance = 78.5688;
			constexpr double heading = 1.95019;
			constexpr double elevation = 8.18631;
			constexpr double grade = -0.0333845;
			constexpr double road_incline_angle = 1.50788;
			constexpr double sine_road_incline_angle = 0.998021;
			constexpr double gravity = 9.80671;
			constexpr double gravity_times_sine_road_incline_angle = 9.7873;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 42.8994;
			constexpr double wind_direction = 4.91125;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 885.153;
			constexpr double air_temp = 13.5678;
			constexpr double pressure = 931.687;
			constexpr double air_density = 1.21762;
			constexpr double reciprocal_speed_of_sound = 0.00292647;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 343.821;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 5") {
		constexpr double drag_coefficient = 0.00677877;
		constexpr double frontal_area = 5.07753;
		constexpr double array_area = 9.91262;
		constexpr double array_efficiency = 18.2529;
		constexpr double energy_capacity = 414.455;
		constexpr double min_voltage = 104.243;
		constexpr double max_voltage = 158.405;
		constexpr double resistance = 0.60238;
		constexpr double hysteresis_loss = 2.24989;
		constexpr double eddy_current_loss_coefficient = 0.0211525;
		constexpr double alpha = 6.87558;
		constexpr double beta = -4.27794;
		constexpr double a = -2.24318;
		constexpr double b = 7.13569e-06;
		constexpr double c = -0.336078;
		constexpr double pressure_at_stc = 152.977;
		constexpr double mass = 797.868;
		constexpr double wheel_radius = 0.286919;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {70.9567, -111.767};
			constexpr GeographicalCoordinate end_coordinate = {18.1813, 173.126};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 38.3484;
			constexpr double weather_station = 8.40883;
			constexpr double distance = 26.3753;
			constexpr double heading = 5.65376;
			constexpr double elevation = -243.76;
			constexpr double grade = 0.645276;
			constexpr double road_incline_angle = 1.51588;
			constexpr double sine_road_incline_angle = 0.998493;
			constexpr double gravity = 9.78221;
			constexpr double gravity_times_sine_road_incline_angle = 9.76747;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 18.2036;
			constexpr double wind_direction = 1.05327;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 883.305;
			constexpr double air_temp = -39.8626;
			constexpr double pressure = 1096.34;
			constexpr double air_density = 1.13366;
			constexpr double reciprocal_speed_of_sound = 0.00308347;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1598.2;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {73.6897, 119.247};
			constexpr GeographicalCoordinate end_coordinate = {-85.5438, 132.426};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 97.8667;
			constexpr double weather_station = 4.67465;
			constexpr double distance = 70.5798;
			constexpr double heading = 5.33635;
			constexpr double elevation = -195.358;
			constexpr double grade = -0.00204682;
			constexpr double road_incline_angle = -0.516698;
			constexpr double sine_road_incline_angle = -0.494012;
			constexpr double gravity = 9.79399;
			constexpr double gravity_times_sine_road_incline_angle = -4.83835;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 40.8165;
			constexpr double wind_direction = 2.04523;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 492.376;
			constexpr double air_temp = 4.54316;
			constexpr double pressure = 969.568;
			constexpr double air_density = 1.12442;
			constexpr double reciprocal_speed_of_sound = 0.0029599;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 890.878;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-84.237, -65.9177};
			constexpr GeographicalCoordinate end_coordinate = {85.9235, 131.89};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 111.991;
			constexpr double weather_station = 8.2534;
			constexpr double distance = 85.2406;
			constexpr double heading = 4.25783;
			constexpr double elevation = 402.327;
			constexpr double grade = -0.940305;
			constexpr double road_incline_angle = -0.284424;
			constexpr double sine_road_incline_angle = -0.280604;
			constexpr double gravity = 9.81985;
			constexpr double gravity_times_sine_road_incline_angle = -2.75549;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 22.8818;
			constexpr double wind_direction = 0.983905;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 613.997;
			constexpr double air_temp = 27.1487;
			constexpr double pressure = 1073.74;
			constexpr double air_density = 1.26819;
			constexpr double reciprocal_speed_of_sound = 0.00308296;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1110.93;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 6") {
		constexpr double drag_coefficient = 0.00218959;
		constexpr double frontal_area = 3.82371;
		constexpr double array_area = 5.7018;
		constexpr double array_efficiency = 16.4078;
		constexpr double energy_capacity = 6143.84;
		constexpr double min_voltage = 103.887;
		constexpr double max_voltage = 133.039;
		constexpr double resistance = 0.290783;
		constexpr double hysteresis_loss = 2.06674;
		constexpr double eddy_current_loss_coefficient = 0.0449454;
		constexpr double alpha = -9.97615;
		constexpr double beta = -8.73385;
		constexpr double a = 7.44124;
		constexpr double b = -4.2692e-06;
		constexpr double c = 0.737102;
		constexpr double pressure_at_stc = 183.462;
		constexpr double mass = 889.638;
		constexpr double wheel_radius = 0.437111;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {79.8688, 66.8653};
			constexpr GeographicalCoordinate end_coordinate = {83.6734, -129.852};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 29.6787;
			constexpr double weather_station = 4.00512;
			constexpr double distance = 82.0746;
			constexpr double heading = 3.17905;
			constexpr double elevation = 159.136;
			constexpr double grade = 0.823532;
			constexpr double road_incline_angle = 1.42715;
			constexpr double sine_road_incline_angle = 0.989701;
			constexpr double gravity = 9.80657;
			constexpr double gravity_times_sine_road_incline_angle = 9.70557;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 10.035;
			constexpr double wind_direction = 1.13174;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 641.037;
			constexpr double air_temp = 39.9252;
			constexpr double pressure = 1052.33;
			constexpr double air_density = 1.06965;
			constexpr double reciprocal_speed_of_sound = 0.00295929;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 599.716;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-79.5095, -153.821};
			constexpr GeographicalCoordinate end_coordinate = {32.2331, 47.0147};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 103.448;
			constexpr double weather_station = 1.28288;
			constexpr double distance = 20.5501;
			constexpr double heading = 0.45207;
			constexpr double elevation = -384.756;
			constexpr double grade = -0.919835;
			constexpr double road_incline_angle = -0.948314;
			constexpr double sine_road_incline_angle = -0.812434;
			constexpr double gravity = 9.79675;
			constexpr double gravity_times_sine_road_incline_angle = -7.95921;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 12.3557;
			constexpr double wind_direction = 4.13392;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 388.814;
			constexpr double air_temp = -15.7772;
			constexpr double pressure = 1024.84;
			constexpr double air_density = 1.27701;
			constexpr double reciprocal_speed_of_sound = 0.00290431;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 363.751;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {21.1221, -29.4799};
			constexpr GeographicalCoordinate end_coordinate = {-76.6535, 25.6046};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 39.3284;
			constexpr double weather_station = 8.44573;
			constexpr double distance = 58.36;
			constexpr double heading = 3.06084;
			constexpr double elevation = 229.645;
			constexpr double grade = 0.532114;
			constexpr double road_incline_angle = -1.41652;
			constexpr double sine_road_incline_angle = -0.988123;
			constexpr double gravity = 9.79007;
			constexpr double gravity_times_sine_road_incline_angle = -9.67379;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 38.4839;
			constexpr double wind_direction = 5.23611;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 28.8519;
			constexpr double air_temp = -24.8995;
			constexpr double pressure = 927.895;
			constexpr double air_density = 1.25558;
			constexpr double reciprocal_speed_of_sound = 0.00296069;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 26.9922;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 7") {
		constexpr double drag_coefficient = 0.000563062;
		constexpr double frontal_area = 1.67347;
		constexpr double array_area = 4.49487;
		constexpr double array_efficiency = 26.4425;
		constexpr double energy_capacity = 7929.5;
		constexpr double min_voltage = 119.137;
		constexpr double max_voltage = 123.305;
		constexpr double resistance = 0.958413;
		constexpr double hysteresis_loss = 2.33828;
		constexpr double eddy_current_loss_coefficient = 0.00870963;
		constexpr double alpha = -5.6572;
		constexpr double beta = -5.88364;
		constexpr double a = 7.99897;
		constexpr double b = -6.17416e-06;
		constexpr double c = -0.538446;
		constexpr double pressure_at_stc = 176.962;
		constexpr double mass = 327.782;
		constexpr double wheel_radius = 0.210374;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-27.2973, 153.435};
			constexpr GeographicalCoordinate end_coordinate = {-72.7919, -138.25};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 185.577;
			constexpr double weather_station = 1.89915;
			constexpr double distance = 5.236;
			constexpr double heading = 2.72533;
			constexpr double elevation = -110.499;
			constexpr double grade = -0.696902;
			constexpr double road_incline_angle = -1.09894;
			constexpr double sine_road_incline_angle = -0.890724;
			constexpr double gravity = 9.7809;
			constexpr double gravity_times_sine_road_incline_angle = -8.71209;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 11.3413;
			constexpr double wind_direction = 5.00237;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 220.117;
			constexpr double air_temp = -1.25045;
			constexpr double pressure = 1046.77;
			constexpr double air_density = 1.0205;
			constexpr double reciprocal_speed_of_sound = 0.00305315;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 261.621;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-34.855, 154.454};
			constexpr GeographicalCoordinate end_coordinate = {33.238, 92.9408};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 77.5401;
			constexpr double weather_station = 1.59859;
			constexpr double distance = 14.5108;
			constexpr double heading = 2.0423;
			constexpr double elevation = -226.367;
			constexpr double grade = -0.899323;
			constexpr double road_incline_angle = -0.800681;
			constexpr double sine_road_incline_angle = -0.71783;
			constexpr double gravity = 9.81659;
			constexpr double gravity_times_sine_road_incline_angle = -7.04664;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 49.1221;
			constexpr double wind_direction = 4.09968;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 679.456;
			constexpr double air_temp = 36.5857;
			constexpr double pressure = 986.475;
			constexpr double air_density = 1.11325;
			constexpr double reciprocal_speed_of_sound = 0.00302643;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 807.57;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-85.9564, -55.9075};
			constexpr GeographicalCoordinate end_coordinate = {-14.5198, -121.792};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 84.4819;
			constexpr double weather_station = 1.57831;
			constexpr double distance = 63.1974;
			constexpr double heading = 0.497689;
			constexpr double elevation = -354.278;
			constexpr double grade = 0.293698;
			constexpr double road_incline_angle = 0.287948;
			constexpr double sine_road_incline_angle = 0.283986;
			constexpr double gravity = 9.7901;
			constexpr double gravity_times_sine_road_incline_angle = 2.78025;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 40.0314;
			constexpr double wind_direction = 5.53996;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 718.106;
			constexpr double air_temp = -10.8976;
			constexpr double pressure = 1051.83;
			constexpr double air_density = 1.11811;
			constexpr double reciprocal_speed_of_sound = 0.00299918;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 853.507;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 8") {
		constexpr double drag_coefficient = 0.0061998;
		constexpr double frontal_area = 5.156;
		constexpr double array_area = 5.18562;
		constexpr double array_efficiency = 23.8574;
		constexpr double energy_capacity = 5725.45;
		constexpr double min_voltage = 81.9343;
		constexpr double max_voltage = 96.1761;
		constexpr double resistance = 0.228567;
		constexpr double hysteresis_loss = 1.84617;
		constexpr double eddy_current_loss_coefficient = 0.0341276;
		constexpr double alpha = 6.28187;
		constexpr double beta = 8.40938;
		constexpr double a = -4.01842;
		constexpr double b = 6.70747e-06;
		constexpr double c = 0.683172;
		constexpr double pressure_at_stc = 104.039;
		constexpr double mass = 276.91;
		constexpr double wheel_radius = 0.23767;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-83.2961, 52.9456};
			constexpr GeographicalCoordinate end_coordinate = {-38.1093, 44.5484};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 147.157;
			constexpr double weather_station = 7.1017;
			constexpr double distance = 23.0043;
			constexpr double heading = 2.23565;
			constexpr double elevation = -82.7926;
			constexpr double grade = -0.455031;
			constexpr double road_incline_angle = 0.820845;
			constexpr double sine_road_incline_angle = 0.731722;
			constexpr double gravity = 9.80163;
			constexpr double gravity_times_sine_road_incline_angle = 7.17207;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 20.3549;
			constexpr double wind_direction = 0.334009;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 57.2413;
			constexpr double air_temp = -45.7417;
			constexpr double pressure = 964.095;
			constexpr double air_density = 1.25419;
			constexpr double reciprocal_speed_of_sound = 0.00303327;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 70.8163;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {75.3403, -68.5656};
			constexpr GeographicalCoordinate end_coordinate = {-72.7321, 5.3378};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 15.9594;
			constexpr double weather_station = 0.923551;
			constexpr double distance = 96.1202;
			constexpr double heading = 3.26104;
			constexpr double elevation = 452.927;
			constexpr double grade = 0.441268;
			constexpr double road_incline_angle = -1.41818;
			constexpr double sine_road_incline_angle = -0.988377;
			constexpr double gravity = 9.78767;
			constexpr double gravity_times_sine_road_incline_angle = -9.67391;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 3.91763;
			constexpr double wind_direction = 1.50438;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 481.233;
			constexpr double air_temp = 10.6064;
			constexpr double pressure = 1054.36;
			constexpr double air_density = 1.15548;
			constexpr double reciprocal_speed_of_sound = 0.00304976;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 595.358;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-70.8231, -47.6556};
			constexpr GeographicalCoordinate end_coordinate = {-64.6241, -58.5059};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 160.524;
			constexpr double weather_station = 2.49935;
			constexpr double distance = 95.2107;
			constexpr double heading = 2.20658;
			constexpr double elevation = 443.749;
			constexpr double grade = 0.797503;
			constexpr double road_incline_angle = -1.11252;
			constexpr double sine_road_incline_angle = -0.896815;
			constexpr double gravity = 9.78621;
			constexpr double gravity_times_sine_road_incline_angle = -8.77642;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 18.8918;
			constexpr double wind_direction = 3.0469;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 383.052;
			constexpr double air_temp = 26.2774;
			constexpr double pressure = 1082.88;
			constexpr double air_density = 1.08773;
			constexpr double reciprocal_speed_of_sound = 0.00299345;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 473.893;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 9") {
		constexpr double drag_coefficient = 0.00274705;
		constexpr double frontal_area = 8.52982;
		constexpr double array_area = 8.66068;
		constexpr double array_efficiency = 28.1102;
		constexpr double energy_capacity = 1482.82;
		constexpr double min_voltage = 75.5449;
		constexpr double max_voltage = 93.9074;
		constexpr double resistance = 0.743476;
		constexpr double hysteresis_loss = 2.28159;
		constexpr double eddy_current_loss_coefficient = 0.0497845;
		constexpr double alpha = 3.39438;
		constexpr double beta = 5.53731;
		constexpr double a = -4.12982;
		constexpr double b = 4.6142e-06;
		constexpr double c = -0.788395;
		constexpr double pressure_at_stc = 178.106;
		constexpr double mass = 168.871;
		constexpr double wheel_radius = 0.352037;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		{  
			constexpr GeographicalCoordinate start_coordinate = {74.9055, -135.105};
			constexpr GeographicalCoordinate end_coordinate = {7.94471, -69.8307};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 140.632;
			constexpr double weather_station = 8.34446;
			constexpr double distance = 65.3419;
			constexpr double heading = 2.23921;
			constexpr double elevation = 476.441;
			constexpr double grade = -0.661137;
			constexpr double road_incline_angle = -0.142196;
			constexpr double sine_road_incline_angle = -0.141717;
			constexpr double gravity = 9.80301;
			constexpr double gravity_times_sine_road_incline_angle = -1.38925;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 41.4462;
			constexpr double wind_direction = 0.761273;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 128.064;
			constexpr double air_temp = 23.9287;
			constexpr double pressure = 902.192;
			constexpr double air_density = 1.17067;
			constexpr double reciprocal_speed_of_sound = 0.00304807;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 311.776;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-15.4587, -44.8522};
			constexpr GeographicalCoordinate end_coordinate = {44.5072, 161.003};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 17.6819;
			constexpr double weather_station = 5.0737;
			constexpr double distance = 81.2315;
			constexpr double heading = 4.61242;
			constexpr double elevation = -357.603;
			constexpr double grade = -0.85799;
			constexpr double road_incline_angle = -1.35589;
			constexpr double sine_road_incline_angle = -0.976996;
			constexpr double gravity = 9.79011;
			constexpr double gravity_times_sine_road_incline_angle = -9.5649;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 22.7938;
			constexpr double wind_direction = 3.6364;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 611.763;
			constexpr double air_temp = -13.9988;
			constexpr double pressure = 1096.83;
			constexpr double air_density = 1.16311;
			constexpr double reciprocal_speed_of_sound = 0.00302362;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 1489.36;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-53.731, 80.1096};
			constexpr GeographicalCoordinate end_coordinate = {-69.6966, -1.95894};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 167.039;
			constexpr double weather_station = 0.473139;
			constexpr double distance = 68.9916;
			constexpr double heading = 0.144451;
			constexpr double elevation = -236.625;
			constexpr double grade = 0.291208;
			constexpr double road_incline_angle = -0.763317;
			constexpr double sine_road_incline_angle = -0.691322;
			constexpr double gravity = 9.81917;
			constexpr double gravity_times_sine_road_incline_angle = -6.7882;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 34.3756;
			constexpr double wind_direction = 3.94876;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 386.751;
			constexpr double air_temp = 49.4628;
			constexpr double pressure = 1006.68;
			constexpr double air_density = 1.02747;
			constexpr double reciprocal_speed_of_sound = 0.00297194;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			const auto runner = RaceSegmentRunner(car);
			const double result = runner.calculate_power_in(route_segment, weather_data);
			constexpr double expected = 941.558;
			REQUIRE_THAT(result, WithinRel(expected, EPSILON));
		}
	}
}
TEST_CASE("RaceSegmentRunner: calculate_power_net", "[RaceSegmentRunner]") {
	SECTION("Random Test 0") {
		constexpr double drag_coefficient = 0.00142098;
		constexpr double frontal_area = 8.9602;
		constexpr double array_area = 6.31195;
		constexpr double array_efficiency = 19.9615;
		constexpr double energy_capacity = 118.554;
		constexpr double min_voltage = 73.6057;
		constexpr double max_voltage = 139.645;
		constexpr double resistance = 0.717542;
		constexpr double hysteresis_loss = 2.71646;
		constexpr double eddy_current_loss_coefficient = 0.023512;
		constexpr double alpha = -1.49446;
		constexpr double beta = 9.01273;
		constexpr double a = -7.33281;
		constexpr double b = -6.73299e-06;
		constexpr double c = -0.508166;
		constexpr double pressure_at_stc = 199.98;
		constexpr double mass = 911.139;
		constexpr double wheel_radius = 0.14754;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-11.3753, 59.9837};
			constexpr GeographicalCoordinate end_coordinate = {-22.3046, -160.735};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 168.913;
			constexpr double weather_station = 6.25485;
			constexpr double distance = 4.25496;
			constexpr double heading = 5.80197;
			constexpr double elevation = 361.628;
			constexpr double grade = -0.150766;
			constexpr double road_incline_angle = -0.69116;
			constexpr double sine_road_incline_angle = -0.637432;
			constexpr double gravity = 9.81449;
			constexpr double gravity_times_sine_road_incline_angle = -6.25607;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 43.8517;
			constexpr double wind_direction = 2.97257;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 640.544;
			constexpr double air_temp = 24.0589;
			constexpr double pressure = 1050.52;
			constexpr double air_density = 1.27318;
			constexpr double reciprocal_speed_of_sound = 0.00300933;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.207517;
			constexpr double speed = 6.01549;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {18.1858, 177.004};
			constexpr GeographicalCoordinate end_coordinate = {30.594, 69.8845};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 59.2209;
			constexpr double weather_station = 0.679109;
			constexpr double distance = 67.7781;
			constexpr double heading = 6.12403;
			constexpr double elevation = -17.3368;
			constexpr double grade = 0.811445;
			constexpr double road_incline_angle = 0.290821;
			constexpr double sine_road_incline_angle = 0.286739;
			constexpr double gravity = 9.79951;
			constexpr double gravity_times_sine_road_incline_angle = 2.8099;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 45.4887;
			constexpr double wind_direction = 5.82134;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 756.581;
			constexpr double air_temp = 1.85419;
			constexpr double pressure = 965.291;
			constexpr double air_density = 1.04332;
			constexpr double reciprocal_speed_of_sound = 0.00298093;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.238614;
			constexpr double speed = 18.6587;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {70.1537, -163.535};
			constexpr GeographicalCoordinate end_coordinate = {79.8619, -14.3196};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 9.35389;
			constexpr double weather_station = 3.52063;
			constexpr double distance = 41.1583;
			constexpr double heading = 0.876279;
			constexpr double elevation = -258.702;
			constexpr double grade = 0.88673;
			constexpr double road_incline_angle = 0.977953;
			constexpr double sine_road_incline_angle = 0.829355;
			constexpr double gravity = 9.81574;
			constexpr double gravity_times_sine_road_incline_angle = 8.14073;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 15.7994;
			constexpr double wind_direction = 5.95746;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 238.273;
			constexpr double air_temp = -11.3547;
			constexpr double pressure = 934.494;
			constexpr double air_density = 1.05976;
			constexpr double reciprocal_speed_of_sound = 0.00304566;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.58664;
			constexpr double speed = 12.9378;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
	}
	SECTION("Random Test 1") {
		constexpr double drag_coefficient = 0.00531873;
		constexpr double frontal_area = 9.76343;
		constexpr double array_area = 5.91076;
		constexpr double array_efficiency = 22.3563;
		constexpr double energy_capacity = 4741.38;
		constexpr double min_voltage = 78.2714;
		constexpr double max_voltage = 161.37;
		constexpr double resistance = 0.121668;
		constexpr double hysteresis_loss = 1.42651;
		constexpr double eddy_current_loss_coefficient = 0.0442719;
		constexpr double alpha = 4.84278;
		constexpr double beta = 7.72768;
		constexpr double a = -3.87306;
		constexpr double b = 2.92636e-06;
		constexpr double c = 0.480345;
		constexpr double pressure_at_stc = 150.769;
		constexpr double mass = 982.317;
		constexpr double wheel_radius = 0.253454;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {27.3442, -66.6725};
			constexpr GeographicalCoordinate end_coordinate = {-74.7091, 133.134};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 20.5207;
			constexpr double weather_station = 0.804636;
			constexpr double distance = 78.7029;
			constexpr double heading = 4.97395;
			constexpr double elevation = -351.238;
			constexpr double grade = -0.0267543;
			constexpr double road_incline_angle = 1.25202;
			constexpr double sine_road_incline_angle = 0.949621;
			constexpr double gravity = 9.81557;
			constexpr double gravity_times_sine_road_incline_angle = 9.32107;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 36.6533;
			constexpr double wind_direction = 4.4118;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 105.697;
			constexpr double air_temp = -13.0331;
			constexpr double pressure = 924.433;
			constexpr double air_density = 1.14583;
			constexpr double reciprocal_speed_of_sound = 0.00309202;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.444271;
			constexpr double speed = 26.8526;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -3.23075e+43;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {40.3251, 8.81015};
			constexpr GeographicalCoordinate end_coordinate = {-61.7519, -29.4762};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 192.386;
			constexpr double weather_station = 1.90127;
			constexpr double distance = 39.8984;
			constexpr double heading = 3.24532;
			constexpr double elevation = -6.66272;
			constexpr double grade = 0.0507716;
			constexpr double road_incline_angle = 1.05173;
			constexpr double sine_road_incline_angle = 0.86828;
			constexpr double gravity = 9.8001;
			constexpr double gravity_times_sine_road_incline_angle = 8.50923;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 25.1558;
			constexpr double wind_direction = 5.13136;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 619.879;
			constexpr double air_temp = -26.3546;
			constexpr double pressure = 992.317;
			constexpr double air_density = 1.26261;
			constexpr double reciprocal_speed_of_sound = 0.00309202;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.410633;
			constexpr double speed = 19.504;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -1.22204e+43;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {31.4904, -30.355};
			constexpr GeographicalCoordinate end_coordinate = {-60.4474, -92.3505};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 156.005;
			constexpr double weather_station = 1.94496;
			constexpr double distance = 6.36703;
			constexpr double heading = 1.64058;
			constexpr double elevation = 11.7125;
			constexpr double grade = -0.529012;
			constexpr double road_incline_angle = -0.532512;
			constexpr double sine_road_incline_angle = -0.507699;
			constexpr double gravity = 9.78057;
			constexpr double gravity_times_sine_road_incline_angle = -4.96558;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 22.301;
			constexpr double wind_direction = 6.08368;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 389.979;
			constexpr double air_temp = 19.2771;
			constexpr double pressure = 967.688;
			constexpr double air_density = 1.17992;
			constexpr double reciprocal_speed_of_sound = 0.00306363;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.532809;
			constexpr double speed = 15.5731;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -6.11977e+42;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 2") {
		constexpr double drag_coefficient = 0.00231875;
		constexpr double frontal_area = 6.36548;
		constexpr double array_area = 6.25191;
		constexpr double array_efficiency = 27.9777;
		constexpr double energy_capacity = 1841.39;
		constexpr double min_voltage = 106.545;
		constexpr double max_voltage = 126.736;
		constexpr double resistance = 0.925623;
		constexpr double hysteresis_loss = 4.675;
		constexpr double eddy_current_loss_coefficient = 0.0222055;
		constexpr double alpha = 5.25251;
		constexpr double beta = 6.22067;
		constexpr double a = -8.6917;
		constexpr double b = 1.52671e-06;
		constexpr double c = -0.490851;
		constexpr double pressure_at_stc = 137.119;
		constexpr double mass = 713.912;
		constexpr double wheel_radius = 0.241218;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {21.0355, -54.6678};
			constexpr GeographicalCoordinate end_coordinate = {-29.5197, 68.9885};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 156.6;
			constexpr double weather_station = 8.1106;
			constexpr double distance = 25.786;
			constexpr double heading = 3.088;
			constexpr double elevation = -33.6768;
			constexpr double grade = 0.926981;
			constexpr double road_incline_angle = -1.00328;
			constexpr double sine_road_incline_angle = -0.843239;
			constexpr double gravity = 9.79211;
			constexpr double gravity_times_sine_road_incline_angle = -8.25708;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 33.1993;
			constexpr double wind_direction = 2.1113;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 737.972;
			constexpr double air_temp = -8.54597;
			constexpr double pressure = 982.117;
			constexpr double air_density = 1.04267;
			constexpr double reciprocal_speed_of_sound = 0.00290808;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.742823;
			constexpr double speed = 19.1588;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-23.9457, 4.91039};
			constexpr GeographicalCoordinate end_coordinate = {-50.7612, 36.3525};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 22.3531;
			constexpr double weather_station = 9.21491;
			constexpr double distance = 50.0831;
			constexpr double heading = 0.816995;
			constexpr double elevation = 284.569;
			constexpr double grade = 0.649609;
			constexpr double road_incline_angle = -1.39194;
			constexpr double sine_road_incline_angle = -0.984048;
			constexpr double gravity = 9.78946;
			constexpr double gravity_times_sine_road_incline_angle = -9.6333;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 30.0296;
			constexpr double wind_direction = 5.78095;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 558.551;
			constexpr double air_temp = 41.1182;
			constexpr double pressure = 1041.99;
			constexpr double air_density = 1.0442;
			constexpr double reciprocal_speed_of_sound = 0.00295392;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.262682;
			constexpr double speed = 18.9626;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-0.463566, 34.692};
			constexpr GeographicalCoordinate end_coordinate = {-22.4824, -141.225};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 160.461;
			constexpr double weather_station = 7.13512;
			constexpr double distance = 31.1611;
			constexpr double heading = 1.12122;
			constexpr double elevation = -49.2188;
			constexpr double grade = -0.0450298;
			constexpr double road_incline_angle = 1.51484;
			constexpr double sine_road_incline_angle = 0.998435;
			constexpr double gravity = 9.79745;
			constexpr double gravity_times_sine_road_incline_angle = 9.78211;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 15.409;
			constexpr double wind_direction = 4.02554;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 502.772;
			constexpr double air_temp = -49.8883;
			constexpr double pressure = 953.165;
			constexpr double air_density = 1.07315;
			constexpr double reciprocal_speed_of_sound = 0.00300347;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.415008;
			constexpr double speed = 17.9109;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
	}
	SECTION("Random Test 3") {
		constexpr double drag_coefficient = 0.00963345;
		constexpr double frontal_area = 6.67444;
		constexpr double array_area = 3.93857;
		constexpr double array_efficiency = 28.6153;
		constexpr double energy_capacity = 7157.89;
		constexpr double min_voltage = 92.9169;
		constexpr double max_voltage = 98.1057;
		constexpr double resistance = 0.340561;
		constexpr double hysteresis_loss = 1.88811;
		constexpr double eddy_current_loss_coefficient = 0.00985866;
		constexpr double alpha = -4.93513;
		constexpr double beta = -0.887302;
		constexpr double a = -6.23832;
		constexpr double b = -1.12351e-06;
		constexpr double c = -0.132361;
		constexpr double pressure_at_stc = 171.6;
		constexpr double mass = 233.438;
		constexpr double wheel_radius = 0.469571;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {65.9917, -56.0782};
			constexpr GeographicalCoordinate end_coordinate = {30.5494, -42.5841};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 11.1243;
			constexpr double weather_station = 9.39393;
			constexpr double distance = 2.37538;
			constexpr double heading = 0.182925;
			constexpr double elevation = 495.109;
			constexpr double grade = 0.0868067;
			constexpr double road_incline_angle = 1.01412;
			constexpr double sine_road_incline_angle = 0.849014;
			constexpr double gravity = 9.78126;
			constexpr double gravity_times_sine_road_incline_angle = 8.30442;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 16.3929;
			constexpr double wind_direction = 3.38443;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 824.091;
			constexpr double air_temp = 43.8098;
			constexpr double pressure = 988.367;
			constexpr double air_density = 1.26287;
			constexpr double reciprocal_speed_of_sound = 0.00306101;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.639444;
			constexpr double speed = 18.1221;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -48617.3;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {51.7363, -120.638};
			constexpr GeographicalCoordinate end_coordinate = {-3.53405, -177.548};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 168.93;
			constexpr double weather_station = 9.44015;
			constexpr double distance = 25.0111;
			constexpr double heading = 2.56475;
			constexpr double elevation = 65.5923;
			constexpr double grade = -0.90303;
			constexpr double road_incline_angle = 1.36683;
			constexpr double sine_road_incline_angle = 0.979271;
			constexpr double gravity = 9.81179;
			constexpr double gravity_times_sine_road_incline_angle = 9.60841;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 44.9958;
			constexpr double wind_direction = 0.513902;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 392.612;
			constexpr double air_temp = -46.1452;
			constexpr double pressure = 960.279;
			constexpr double air_density = 1.18576;
			constexpr double reciprocal_speed_of_sound = 0.00296172;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.510845;
			constexpr double speed = 14.8391;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -46560.5;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-80.2056, 21.1424};
			constexpr GeographicalCoordinate end_coordinate = {47.1301, 64.8714};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 80.3423;
			constexpr double weather_station = 5.44638;
			constexpr double distance = 97.5734;
			constexpr double heading = 1.20858;
			constexpr double elevation = 445.329;
			constexpr double grade = 0.94584;
			constexpr double road_incline_angle = -0.655829;
			constexpr double sine_road_incline_angle = -0.609816;
			constexpr double gravity = 9.80777;
			constexpr double gravity_times_sine_road_incline_angle = -5.98094;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 3.25726;
			constexpr double wind_direction = 5.15526;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 967.297;
			constexpr double air_temp = 6.31733;
			constexpr double pressure = 972.246;
			constexpr double air_density = 1.22792;
			constexpr double reciprocal_speed_of_sound = 0.00301841;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.220748;
			constexpr double speed = 17.5988;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
	}
	SECTION("Random Test 4") {
		constexpr double drag_coefficient = 0.00922505;
		constexpr double frontal_area = 2.23815;
		constexpr double array_area = 4.69067;
		constexpr double array_efficiency = 27.701;
		constexpr double energy_capacity = 1297.13;
		constexpr double min_voltage = 93.9856;
		constexpr double max_voltage = 168.441;
		constexpr double resistance = 0.593223;
		constexpr double hysteresis_loss = 2.80219;
		constexpr double eddy_current_loss_coefficient = 0.0132347;
		constexpr double alpha = 4.60207;
		constexpr double beta = 1.29203;
		constexpr double a = 8.52266;
		constexpr double b = 3.9538e-07;
		constexpr double c = -0.115345;
		constexpr double pressure_at_stc = 111.152;
		constexpr double mass = 567.915;
		constexpr double wheel_radius = 0.128546;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {68.317, -20.426};
			constexpr GeographicalCoordinate end_coordinate = {-59.1176, -114.799};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 116.578;
			constexpr double weather_station = 9.33963;
			constexpr double distance = 64.5145;
			constexpr double heading = 3.25925;
			constexpr double elevation = -84.4901;
			constexpr double grade = 0.134646;
			constexpr double road_incline_angle = 1.12475;
			constexpr double sine_road_incline_angle = 0.90216;
			constexpr double gravity = 9.78659;
			constexpr double gravity_times_sine_road_incline_angle = 8.82907;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 10.5502;
			constexpr double wind_direction = 4.99317;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 683.894;
			constexpr double air_temp = 30.2848;
			constexpr double pressure = 1097.89;
			constexpr double air_density = 1.13556;
			constexpr double reciprocal_speed_of_sound = 0.00293586;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.0530574;
			constexpr double speed = 0.962136;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -1.78874e+15;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-77.1996, -168.35};
			constexpr GeographicalCoordinate end_coordinate = {18.3247, -4.34307};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 75.0367;
			constexpr double weather_station = 3.0037;
			constexpr double distance = 16.6022;
			constexpr double heading = 4.9007;
			constexpr double elevation = -428.378;
			constexpr double grade = -0.662873;
			constexpr double road_incline_angle = -0.351321;
			constexpr double sine_road_incline_angle = -0.344139;
			constexpr double gravity = 9.81839;
			constexpr double gravity_times_sine_road_incline_angle = -3.37889;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 46.1918;
			constexpr double wind_direction = 2.73075;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 297.354;
			constexpr double air_temp = -46.4206;
			constexpr double pressure = 1034.72;
			constexpr double air_density = 1.11653;
			constexpr double reciprocal_speed_of_sound = 0.00306734;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.634722;
			constexpr double speed = 6.71668;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {57.7855, -91.1978};
			constexpr GeographicalCoordinate end_coordinate = {16.9426, 2.71713};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 111.905;
			constexpr double weather_station = 3.17111;
			constexpr double distance = 44.9408;
			constexpr double heading = 2.50389;
			constexpr double elevation = -53.6516;
			constexpr double grade = 0.571524;
			constexpr double road_incline_angle = 0.894816;
			constexpr double sine_road_incline_angle = 0.780094;
			constexpr double gravity = 9.80738;
			constexpr double gravity_times_sine_road_incline_angle = 7.65068;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 27.4569;
			constexpr double wind_direction = 3.64874;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 886.612;
			constexpr double air_temp = -44.4797;
			constexpr double pressure = 1040.71;
			constexpr double air_density = 1.24113;
			constexpr double reciprocal_speed_of_sound = 0.00300563;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.562056;
			constexpr double speed = 6.09169;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
	}
	SECTION("Random Test 5") {
		constexpr double drag_coefficient = 0.00832189;
		constexpr double frontal_area = 2.92876;
		constexpr double array_area = 9.26599;
		constexpr double array_efficiency = 22.9054;
		constexpr double energy_capacity = 7083.18;
		constexpr double min_voltage = 101.298;
		constexpr double max_voltage = 116.077;
		constexpr double resistance = 0.439496;
		constexpr double hysteresis_loss = 3.0296;
		constexpr double eddy_current_loss_coefficient = 0.00665916;
		constexpr double alpha = -2.1839;
		constexpr double beta = -7.07941;
		constexpr double a = -8.311;
		constexpr double b = 4.62131e-07;
		constexpr double c = 0.506016;
		constexpr double pressure_at_stc = 150.083;
		constexpr double mass = 196.676;
		constexpr double wheel_radius = 0.306486;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-5.63621, 45.4456};
			constexpr GeographicalCoordinate end_coordinate = {-53.7219, 145.453};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 110.497;
			constexpr double weather_station = 3.44768;
			constexpr double distance = 71.087;
			constexpr double heading = 3.46185;
			constexpr double elevation = 447.065;
			constexpr double grade = 0.219858;
			constexpr double road_incline_angle = -0.359409;
			constexpr double sine_road_incline_angle = -0.351721;
			constexpr double gravity = 9.79436;
			constexpr double gravity_times_sine_road_incline_angle = -3.44488;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 49.02;
			constexpr double wind_direction = 4.53554;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 107.055;
			constexpr double air_temp = 17.7703;
			constexpr double pressure = 986.859;
			constexpr double air_density = 1.14793;
			constexpr double reciprocal_speed_of_sound = 0.00297642;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.44207;
			constexpr double speed = 4.92563;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = 2851.91;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-50.403, 161.737};
			constexpr GeographicalCoordinate end_coordinate = {-18.2861, 138.737};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 53.645;
			constexpr double weather_station = 5.29575;
			constexpr double distance = 1.34504;
			constexpr double heading = 3.04974;
			constexpr double elevation = -466.286;
			constexpr double grade = -0.907579;
			constexpr double road_incline_angle = -0.359762;
			constexpr double sine_road_incline_angle = -0.352051;
			constexpr double gravity = 9.78723;
			constexpr double gravity_times_sine_road_incline_angle = -3.44561;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 0.424169;
			constexpr double wind_direction = 3.0048;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 372.153;
			constexpr double air_temp = -34.9729;
			constexpr double pressure = 923.321;
			constexpr double air_density = 1.2818;
			constexpr double reciprocal_speed_of_sound = 0.00302509;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.661564;
			constexpr double speed = 15.5741;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {23.0365, -53.0194};
			constexpr GeographicalCoordinate end_coordinate = {-43.49, -100.292};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 115.025;
			constexpr double weather_station = 7.26921;
			constexpr double distance = 92.0149;
			constexpr double heading = 1.62621;
			constexpr double elevation = 173.627;
			constexpr double grade = -0.178841;
			constexpr double road_incline_angle = -0.794737;
			constexpr double sine_road_incline_angle = -0.713679;
			constexpr double gravity = 9.78767;
			constexpr double gravity_times_sine_road_incline_angle = -6.98526;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 38.3669;
			constexpr double wind_direction = 0.766932;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 199.05;
			constexpr double air_temp = -16.7391;
			constexpr double pressure = 983.309;
			constexpr double air_density = 1.10061;
			constexpr double reciprocal_speed_of_sound = 0.0030212;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.731412;
			constexpr double speed = 7.29052;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
	}
	SECTION("Random Test 6") {
		constexpr double drag_coefficient = 0.00944673;
		constexpr double frontal_area = 8.37677;
		constexpr double array_area = 5.78978;
		constexpr double array_efficiency = 19.7502;
		constexpr double energy_capacity = 7531.73;
		constexpr double min_voltage = 82.4546;
		constexpr double max_voltage = 93.5857;
		constexpr double resistance = 0.939815;
		constexpr double hysteresis_loss = 3.22297;
		constexpr double eddy_current_loss_coefficient = 0.0322522;
		constexpr double alpha = -2.5773;
		constexpr double beta = 0.725627;
		constexpr double a = 3.1177;
		constexpr double b = 8.64916e-06;
		constexpr double c = -0.471333;
		constexpr double pressure_at_stc = 111.532;
		constexpr double mass = 202.484;
		constexpr double wheel_radius = 0.389719;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-2.98563, 157.983};
			constexpr GeographicalCoordinate end_coordinate = {-46.6985, 106.417};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 112.045;
			constexpr double weather_station = 2.95124;
			constexpr double distance = 55.0525;
			constexpr double heading = 0.923632;
			constexpr double elevation = 355.535;
			constexpr double grade = -0.213739;
			constexpr double road_incline_angle = -0.230832;
			constexpr double sine_road_incline_angle = -0.228787;
			constexpr double gravity = 9.78728;
			constexpr double gravity_times_sine_road_incline_angle = -2.2392;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 11.3379;
			constexpr double wind_direction = 5.37409;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 219.119;
			constexpr double air_temp = -22.8346;
			constexpr double pressure = 973.043;
			constexpr double air_density = 1.00564;
			constexpr double reciprocal_speed_of_sound = 0.00293574;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.60463;
			constexpr double speed = 25.9451;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {2.09668, -172.114};
			constexpr GeographicalCoordinate end_coordinate = {49.3066, -45.8223};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 154.481;
			constexpr double weather_station = 6.63701;
			constexpr double distance = 27.4912;
			constexpr double heading = 2.1016;
			constexpr double elevation = -397.895;
			constexpr double grade = 0.552233;
			constexpr double road_incline_angle = -0.804223;
			constexpr double sine_road_incline_angle = -0.720292;
			constexpr double gravity = 9.80405;
			constexpr double gravity_times_sine_road_incline_angle = -7.06178;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 20.8192;
			constexpr double wind_direction = 3.90026;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 631.781;
			constexpr double air_temp = -34.1807;
			constexpr double pressure = 979.024;
			constexpr double air_density = 1.25133;
			constexpr double reciprocal_speed_of_sound = 0.00300638;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.621939;
			constexpr double speed = 8.14351;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			REQUIRE_FALSE(result.has_value());
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-23.1259, -80.4567};
			constexpr GeographicalCoordinate end_coordinate = {-83.1865, 146.663};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 139.964;
			constexpr double weather_station = 6.63386;
			constexpr double distance = 53.3091;
			constexpr double heading = 1.08429;
			constexpr double elevation = 454.511;
			constexpr double grade = 0.796765;
			constexpr double road_incline_angle = 0.00398653;
			constexpr double sine_road_incline_angle = 0.00398652;
			constexpr double gravity = 9.80783;
			constexpr double gravity_times_sine_road_incline_angle = 0.0390991;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 15.1094;
			constexpr double wind_direction = 4.6586;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 188.256;
			constexpr double air_temp = 1.67194;
			constexpr double pressure = 918.396;
			constexpr double air_density = 1.28963;
			constexpr double reciprocal_speed_of_sound = 0.00300148;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.908459;
			constexpr double speed = 11.8552;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = 131.031;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 7") {
		constexpr double drag_coefficient = 0.00422374;
		constexpr double frontal_area = 7.43961;
		constexpr double array_area = 3.56516;
		constexpr double array_efficiency = 29.8167;
		constexpr double energy_capacity = 6463.11;
		constexpr double min_voltage = 83.2791;
		constexpr double max_voltage = 107.066;
		constexpr double resistance = 0.47868;
		constexpr double hysteresis_loss = 1.94386;
		constexpr double eddy_current_loss_coefficient = 0.00139881;
		constexpr double alpha = 9.09568;
		constexpr double beta = 0.716784;
		constexpr double a = 9.49673;
		constexpr double b = 6.60599e-06;
		constexpr double c = 0.754454;
		constexpr double pressure_at_stc = 180.646;
		constexpr double mass = 525.676;
		constexpr double wheel_radius = 0.43173;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {-17.832, -34.1862};
			constexpr GeographicalCoordinate end_coordinate = {-59.5189, -114.045};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 129.037;
			constexpr double weather_station = 8.42282;
			constexpr double distance = 59.312;
			constexpr double heading = 3.28071;
			constexpr double elevation = -455.642;
			constexpr double grade = 0.704983;
			constexpr double road_incline_angle = -0.185765;
			constexpr double sine_road_incline_angle = -0.184698;
			constexpr double gravity = 9.78475;
			constexpr double gravity_times_sine_road_incline_angle = -1.80722;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 29.933;
			constexpr double wind_direction = 3.70776;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 837.708;
			constexpr double air_temp = 40.7322;
			constexpr double pressure = 986.418;
			constexpr double air_density = 1.27291;
			constexpr double reciprocal_speed_of_sound = 0.00296386;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.327026;
			constexpr double speed = 27.129;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -8.21902e+28;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {-72.8009, 97.0684};
			constexpr GeographicalCoordinate end_coordinate = {62.9492, -11.3099};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 79.1636;
			constexpr double weather_station = 5.25908;
			constexpr double distance = 89.9315;
			constexpr double heading = 5.01107;
			constexpr double elevation = 222.524;
			constexpr double grade = 0.26544;
			constexpr double road_incline_angle = -0.385072;
			constexpr double sine_road_incline_angle = -0.375626;
			constexpr double gravity = 9.81297;
			constexpr double gravity_times_sine_road_incline_angle = -3.686;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 3.9534;
			constexpr double wind_direction = 3.09927;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 667.574;
			constexpr double air_temp = -9.17127;
			constexpr double pressure = 917.3;
			constexpr double air_density = 1.2057;
			constexpr double reciprocal_speed_of_sound = 0.00309835;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.517053;
			constexpr double speed = 18.8752;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -2.77777e+28;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {66.1842, -136.069};
			constexpr GeographicalCoordinate end_coordinate = {75.0111, -166.189};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 12.1932;
			constexpr double weather_station = 3.2836;
			constexpr double distance = 77.8833;
			constexpr double heading = 4.207;
			constexpr double elevation = -372.247;
			constexpr double grade = -0.0596837;
			constexpr double road_incline_angle = 0.566862;
			constexpr double sine_road_incline_angle = 0.536987;
			constexpr double gravity = 9.81089;
			constexpr double gravity_times_sine_road_incline_angle = 5.26832;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 23.8816;
			constexpr double wind_direction = 4.0778;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 180.853;
			constexpr double air_temp = -5.49068;
			constexpr double pressure = 924.611;
			constexpr double air_density = 1.29547;
			constexpr double reciprocal_speed_of_sound = 0.00290121;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.191283;
			constexpr double speed = 12.8374;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -8.76516e+27;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 8") {
		constexpr double drag_coefficient = 0.00547629;
		constexpr double frontal_area = 4.25578;
		constexpr double array_area = 6.11674;
		constexpr double array_efficiency = 27.6236;
		constexpr double energy_capacity = 893.418;
		constexpr double min_voltage = 147.92;
		constexpr double max_voltage = 165.324;
		constexpr double resistance = 0.869006;
		constexpr double hysteresis_loss = 2.03018;
		constexpr double eddy_current_loss_coefficient = 0.0134115;
		constexpr double alpha = 8.99491;
		constexpr double beta = -2.35715;
		constexpr double a = -3.46516;
		constexpr double b = -9.1706e-06;
		constexpr double c = 0.472314;
		constexpr double pressure_at_stc = 124.405;
		constexpr double mass = 256.367;
		constexpr double wheel_radius = 0.297267;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {84.393, 34.7985};
			constexpr GeographicalCoordinate end_coordinate = {20.2281, 106.369};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 34.7573;
			constexpr double weather_station = 7.25238;
			constexpr double distance = 13.4453;
			constexpr double heading = 3.99585;
			constexpr double elevation = 468.115;
			constexpr double grade = 0.848762;
			constexpr double road_incline_angle = -1.54698;
			constexpr double sine_road_incline_angle = -0.999716;
			constexpr double gravity = 9.81674;
			constexpr double gravity_times_sine_road_incline_angle = -9.81396;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 12.7829;
			constexpr double wind_direction = 5.37977;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 232.586;
			constexpr double air_temp = 19.7025;
			constexpr double pressure = 1070.64;
			constexpr double air_density = 1.14183;
			constexpr double reciprocal_speed_of_sound = 0.00306587;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.913553;
			constexpr double speed = 5.95296;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -6.8149e+15;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {86.2763, -164.119};
			constexpr GeographicalCoordinate end_coordinate = {-84.6842, 99.5959};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 194.936;
			constexpr double weather_station = 9.03085;
			constexpr double distance = 47.3681;
			constexpr double heading = 3.74348;
			constexpr double elevation = -421.048;
			constexpr double grade = 0.80097;
			constexpr double road_incline_angle = 1.15376;
			constexpr double sine_road_incline_angle = 0.914295;
			constexpr double gravity = 9.78257;
			constexpr double gravity_times_sine_road_incline_angle = 8.94415;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 40.2707;
			constexpr double wind_direction = 0.630369;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 244.741;
			constexpr double air_temp = -19.3985;
			constexpr double pressure = 965.335;
			constexpr double air_density = 1.1978;
			constexpr double reciprocal_speed_of_sound = 0.00309326;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.159731;
			constexpr double speed = 17.5563;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -1.78784e+17;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {19.8858, -148.978};
			constexpr GeographicalCoordinate end_coordinate = {-62.8719, 21.0672};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 143.067;
			constexpr double weather_station = 9.09177;
			constexpr double distance = 8.81017;
			constexpr double heading = 0.162479;
			constexpr double elevation = 128.183;
			constexpr double grade = -0.790592;
			constexpr double road_incline_angle = -0.264308;
			constexpr double sine_road_incline_angle = -0.261241;
			constexpr double gravity = 9.81395;
			constexpr double gravity_times_sine_road_incline_angle = -2.56381;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 27.7538;
			constexpr double wind_direction = 3.69776;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 478.688;
			constexpr double air_temp = -12.1639;
			constexpr double pressure = 1019.28;
			constexpr double air_density = 1.07323;
			constexpr double reciprocal_speed_of_sound = 0.00291122;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.840407;
			constexpr double speed = 5.49534;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -5.34948e+15;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
	}
	SECTION("Random Test 9") {
		constexpr double drag_coefficient = 0.0026602;
		constexpr double frontal_area = 4.1315;
		constexpr double array_area = 4.68123;
		constexpr double array_efficiency = 29.0862;
		constexpr double energy_capacity = 6656.86;
		constexpr double min_voltage = 95.4608;
		constexpr double max_voltage = 110.799;
		constexpr double resistance = 0.556289;
		constexpr double hysteresis_loss = 3.686;
		constexpr double eddy_current_loss_coefficient = 0.00145579;
		constexpr double alpha = 1.03508;
		constexpr double beta = 2.1404;
		constexpr double a = 9.55372;
		constexpr double b = -6.25665e-06;
		constexpr double c = 0.340554;
		constexpr double pressure_at_stc = 146.585;
		constexpr double mass = 896.082;
		constexpr double wheel_radius = 0.185326;
		const auto aerobody = Aerobody(drag_coefficient, frontal_area);
		const auto array = Array(array_area, array_efficiency);
		const auto battery = Battery(energy_capacity, resistance, min_voltage, max_voltage);
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto tire = Tire(SaeJ2452Coefficients{alpha, beta, a, b, c}, pressure_at_stc);
		const SolarCar car(aerobody, array, battery, motor, tire, mass, wheel_radius);
		const auto runner = RaceSegmentRunner(car);
		{  
			constexpr GeographicalCoordinate start_coordinate = {48.0707, 72.4883};
			constexpr GeographicalCoordinate end_coordinate = {20.6406, 46.1489};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 57.8034;
			constexpr double weather_station = 0.587037;
			constexpr double distance = 39.0543;
			constexpr double heading = 2.76418;
			constexpr double elevation = 281.409;
			constexpr double grade = 0.880653;
			constexpr double road_incline_angle = -0.214111;
			constexpr double sine_road_incline_angle = -0.212479;
			constexpr double gravity = 9.78863;
			constexpr double gravity_times_sine_road_incline_angle = -2.07988;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 42.3176;
			constexpr double wind_direction = 1.68274;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 762.074;
			constexpr double air_temp = 24.7694;
			constexpr double pressure = 941.056;
			constexpr double air_density = 1.29523;
			constexpr double reciprocal_speed_of_sound = 0.00299006;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.221552;
			constexpr double speed = 10.7885;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -1.55028e+14;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {6.55922, -164.357};
			constexpr GeographicalCoordinate end_coordinate = {19.9568, 62.5442};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 26.358;
			constexpr double weather_station = 3.06295;
			constexpr double distance = 17.8219;
			constexpr double heading = 4.03804;
			constexpr double elevation = -487.671;
			constexpr double grade = 0.846106;
			constexpr double road_incline_angle = 0.960418;
			constexpr double sine_road_incline_angle = 0.819431;
			constexpr double gravity = 9.81651;
			constexpr double gravity_times_sine_road_incline_angle = 8.04396;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 4.17126;
			constexpr double wind_direction = 2.66106;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 324.483;
			constexpr double air_temp = 36.8227;
			constexpr double pressure = 932.421;
			constexpr double air_density = 1.25861;
			constexpr double reciprocal_speed_of_sound = 0.0029813;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.304272;
			constexpr double speed = 28.1731;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -2.7344e+15;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
		{  
			constexpr GeographicalCoordinate start_coordinate = {16.664, -73.8758};
			constexpr GeographicalCoordinate end_coordinate = {-13.0232, 132.626};
			constexpr SegmentEndCondition end_condition = SegmentEndCondition::END_OF_RACE;
			constexpr SegmentType type = SegmentType::RACE;
			constexpr double speed_limit = 43.0478;
			constexpr double weather_station = 6.02414;
			constexpr double distance = 7.62921;
			constexpr double heading = 0.473794;
			constexpr double elevation = 17.1697;
			constexpr double grade = -0.425466;
			constexpr double road_incline_angle = 1.45503;
			constexpr double sine_road_incline_angle = 0.993307;
			constexpr double gravity = 9.81078;
			constexpr double gravity_times_sine_road_incline_angle = 9.74512;
			const RouteSegment route_segment{
						.coordinate_start = start_coordinate,
						.coordinate_end = end_coordinate,
						.end_condition = end_condition,
						.type = type,
						.speed_limit = speed_limit,
						.weather_station = weather_station,
						.distance = distance,
						.heading = heading,
						.elevation = elevation,
						.grade = grade,
						.road_incline_angle = road_incline_angle,
						.sine_road_incline_angle = sine_road_incline_angle,
						.gravity = gravity,
						.gravity_times_sine_road_incline_angle = gravity_times_sine_road_incline_angle,
			};
			constexpr double wind_speed = 18.3311;
			constexpr double wind_direction = 2.6695;
			const auto wind = VelocityVector::from_polar_components(wind_speed, wind_direction);
			constexpr double irradiance = 667.048;
			constexpr double air_temp = -36.3689;
			constexpr double pressure = 937.34;
			constexpr double air_density = 1.19092;
			constexpr double reciprocal_speed_of_sound = 0.00294734;
			const WeatherDataPoint weather_data{
						.wind = wind,
						.irradiance = irradiance,
						.air_temp = air_temp,
						.pressure = pressure,
						.air_density = air_density,
						.reciprocal_speed_of_sound = reciprocal_speed_of_sound,
			};
			constexpr double state_of_charge = 0.0931195;
			constexpr double speed = 14.1315;

			const auto result = runner.calculate_power_net(route_segment, weather_data, state_of_charge, speed);
			constexpr double expected = -3.47438e+14;
			REQUIRE(result.has_value());
			REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
		}
	}
}
