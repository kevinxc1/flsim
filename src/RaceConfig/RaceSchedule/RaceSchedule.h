#ifndef MINISIM_RACESCHEDULE_H
#define MINISIM_RACESCHEDULE_H

#include <string>

#include "ConfigFile/ConfigFile.h"

struct SingleDaySchedule {
	/// The name of the schedule (typically the day on race)
	std::string schedule_name;
	/// The Time that the car will start charging before the day's driving begins
	/// Units: Epoch Time
	double morning_charging_start_time;
	/// The Time that the car will finish charging before the day's driving begins
	/// Units: Epoch Time
	double morning_charging_end_time;
	/// The Time that the race starts
	/// Units: Epoch Time
	double race_start_time;
	/// The Time that the race ends
	/// Units: Epoch Time
	double race_end_time;
	/// The Time that the car will start charging after the day's driving begins
	/// Units: Epoch Time
	double evening_charging_start_time;
	/// The Time that the car will finish charging after the day's driving begins
	/// Units: Epoch Time
	double evening_charging_end_time;
};

class RaceSchedule {
   private:
	/// The name of the schedule (to differentiate when running different potential schedules
	std::string schedule_name;
	std::vector<SingleDaySchedule> schedules;

   public:
	RaceSchedule() = default;
	explicit RaceSchedule(const ConfigFile& schedule_file);

	const std::string& get_schedule_name() const;

	void from_config_file(const ConfigFile& schedule_json, SingleDaySchedule& schedule) const;

	ConfigFile get_config_file() const;

	const SingleDaySchedule& at(size_t index) const;
	const SingleDaySchedule& operator[](size_t index) const;
	size_t size() const;
};

#endif  // MINISIM_RACESCHEDULE_H
