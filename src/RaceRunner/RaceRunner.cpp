#include "RaceRunner.h"

#include "RaceSegmentRunner/RaceSegmentRunner.h"
#include "SolarCar/Battery/BatteryState.h"

constexpr double STATIC_CHARGING_TIME_INCREMENT = 300.0;   
constexpr double CHECKPOINT_DURATION = 1800.0;             

namespace RaceRunner {

double calculate_static_charging_gain(
	const SolarCar& car, const Weather& weather, double weather_station, double start_time, double end_time) {

	double total_energy = 0.0;   

	 
	for (double current_time = start_time; current_time < end_time; current_time += STATIC_CHARGING_TIME_INCREMENT) {
		double time_end = std::min(current_time + STATIC_CHARGING_TIME_INCREMENT, end_time);
		double time_delta = time_end - current_time;

		 
		WeatherDataPoint weather_data = weather.get_weather_during(weather_station, current_time, time_end);

		 
		 
		double power = car.array.power_in(weather_data.irradiance);

		 
		double energy = power * time_delta / 3600.0;
		total_energy += energy;
	}

	return total_energy;
}

std::optional<double> calculate_racetime(
	const SolarCar& car, const Route& route, const Weather& weather, const RaceSchedule& schedule, double speed) {

	 
	BatteryState battery_state(car.battery.get_capacity());   

	 
	RaceSegmentRunner runner(car);

	double total_racetime = 0.0;   
	size_t current_segment_index = 0;
	const size_t total_segments = route.get_num_segments();
	double remaining_segment_distance = 0.0;   

	 
	size_t current_day = 0;
	const SingleDaySchedule& day_schedule = schedule[current_day];
	double current_time = day_schedule.race_start_time;

	while (current_segment_index < total_segments) {
		const RouteSegment& segment = route.get_segment(current_segment_index);
		const SingleDaySchedule& today = schedule[current_day];

		 
		double segment_distance = (remaining_segment_distance > 0.0) ? remaining_segment_distance : segment.distance;
		remaining_segment_distance = 0.0;   

		 
		if (current_time >= today.race_end_time) {
			 
			double evening_charging_gain = calculate_static_charging_gain(
				car, weather, segment.weather_station,
				today.evening_charging_start_time, today.evening_charging_end_time
			);
			battery_state.update_energy_remaining(evening_charging_gain);

			 
			current_day++;
			if (current_day >= schedule.size()) {
				 
				return std::nullopt;
			}

			const SingleDaySchedule& tomorrow = schedule[current_day];

			 
			double morning_charging_gain = calculate_static_charging_gain(
				car, weather, segment.weather_station,
				tomorrow.morning_charging_start_time, tomorrow.morning_charging_end_time
			);
			battery_state.update_energy_remaining(morning_charging_gain);

			 
			current_time = tomorrow.race_start_time;
			continue;
		}

		 
		double segment_time = segment_distance / speed;   
		double segment_end_time = current_time + segment_time;

		 
		if (segment_end_time > today.race_end_time) {
			 
			double time_available = today.race_end_time - current_time;
			double distance_driven = speed * time_available;
			remaining_segment_distance = segment_distance - distance_driven;

			segment_end_time = today.race_end_time;
			segment_time = time_available;
		}

		 
		WeatherDataPoint weather_data = weather.get_weather_during(
			segment.weather_station, current_time, segment_end_time
		);

		 
		double state_of_charge = car.battery.state_of_charge(battery_state.get_energy_remaining());

		 
		auto net_power_optional = runner.calculate_power_net(segment, weather_data, state_of_charge, speed);

		if (!net_power_optional.has_value()) {
			 
			return std::nullopt;
		}

		double net_power = net_power_optional.value();   

		 
		double energy_change = net_power * segment_time / 3600.0;

		 
		battery_state.update_energy_remaining(energy_change);

		 
		if (battery_state.get_energy_remaining() < 0) {
			return std::nullopt;   
		}

		 
		total_racetime += segment_time;
		current_time = segment_end_time;

		 
		if (remaining_segment_distance == 0.0 && segment.end_condition == SegmentEndCondition::CONTROL_STOP) {
			 
			 
			 
			if (current_time < today.race_end_time) {
				double checkpoint_start = current_time;
				double checkpoint_end = current_time + CHECKPOINT_DURATION;

				double checkpoint_energy = calculate_static_charging_gain(
					car, weather, segment.weather_station,
					checkpoint_start, checkpoint_end
				);
				battery_state.update_energy_remaining(checkpoint_energy);

				total_racetime += CHECKPOINT_DURATION;
				current_time = checkpoint_end;
			}
		}

		 
		if (remaining_segment_distance == 0.0) {
			current_segment_index++;
		}
	}

	return total_racetime;
}

}   
