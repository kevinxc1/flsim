#ifndef MINISIM_RACESCHEDULECONSTANTS_H
#define MINISIM_RACESCHEDULECONSTANTS_H

#include <string>

namespace race_schedule {
	const std::string CN_SCHEDULE_FILE_DIRECTORY = "schedule_file_directory";
	const std::string CN_SCHEDULE_LIST = "schedule";
	const std::string CN_SCHEDULE_NAME = "schedule_name";
	namespace schedule_day {
		const std::string CN_NAME = "name";
		const std::string CN_MORNING_CHARGING_START_TIME = "bod_charge_start";
		const std::string CN_MORNING_CHARGING_END_TIME = "bod_charge_end";
		const std::string CN_RACE_START_TIME = "race_start";
		const std::string CN_RACE_END_TIME = "race_end";
		const std::string CN_EVENING_CHARGING_START_TIME = "eod_charge_start";
		const std::string CN_EVENING_CHARGING_END_TIME = "eod_charge_end";
	}  // namespace schedule_day
}  // namespace race_schedule

#endif  // MINISIM_RACESCHEDULECONSTANTS_H
