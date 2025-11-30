#include "RaceSchedule.h"

#include <fstream>
#include <iostream>
#include <string_view>

#include "ConfigFile/ConfigFile.h"
#include "RaceScheduleConstants.h"
#include "Tools/TimeTools.h"

void RaceSchedule::from_config_file(const ConfigFile& schedule_config, SingleDaySchedule& schedule) const {
	struct TimeToSet {
		double& time;
		std::string_view key;
	};

	std::vector<TimeToSet> times_to_set = {
		{schedule.morning_charging_start_time, race_schedule::schedule_day::CN_MORNING_CHARGING_START_TIME},
		{schedule.morning_charging_end_time,   race_schedule::schedule_day::CN_MORNING_CHARGING_END_TIME  },
		{schedule.race_start_time,             race_schedule::schedule_day::CN_RACE_START_TIME            },
		{schedule.race_end_time,               race_schedule::schedule_day::CN_RACE_END_TIME              },
		{schedule.evening_charging_start_time, race_schedule::schedule_day::CN_EVENING_CHARGING_START_TIME},
		{schedule.evening_charging_end_time,   race_schedule::schedule_day::CN_EVENING_CHARGING_END_TIME  }
    };

	for (auto& time_to_set : times_to_set) {
		auto time_string = schedule_config.get_force<std::string>(time_to_set.key.data());
		time_to_set.time = static_cast<double>(parse_time(time_string));
	}
}

RaceSchedule::RaceSchedule(const ConfigFile& schedule_file) {
	schedule_name = schedule_file.get_force<std::string>(race_schedule::CN_SCHEDULE_NAME.data());

	auto schedule_day_list = schedule_file.get_array_force<ConfigFile>(race_schedule::CN_SCHEDULE_LIST.data());
	for (auto& daySchedule : schedule_day_list) {
		SingleDaySchedule day;
		from_config_file(daySchedule, day);
		schedules.push_back(day);
	}
}

const std::string& RaceSchedule::get_schedule_name() const {
	return schedule_name;
}

const SingleDaySchedule& RaceSchedule::at(size_t index) const {
	if (index >= schedules.size()) {
		throw std::exception();
	}
	return schedules[index];
}

const SingleDaySchedule& RaceSchedule::operator[](size_t index) const {
	return at(index);
}

size_t RaceSchedule::size() const {
	return schedules.size();
}
