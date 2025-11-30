#ifndef MINISIM_RACECONFIGCONSTANTS_H
#define MINISIM_RACECONFIGCONSTANTS_H

#include <string_view>

#include "Tools/Conversions.h"

namespace race_config {
	namespace race_schedule {
		constexpr std::string_view CN_SCHEDULE_FILE_DIRECTORY = "schedule_file_directory";
		constexpr std::string_view CN_SCHEDULE_LIST = "schedule";
		constexpr std::string_view CN_SCHEDULE_NAME = "schedule_name";
		namespace schedule_day {
			constexpr std::string_view CN_NAME = "name";
			constexpr std::string_view CN_MORNING_CHARGING_START_TIME = "bod_charge_start";
			constexpr std::string_view CN_MORNING_CHARGING_END_TIME = "bod_charge_end";
			constexpr std::string_view CN_RACE_START_TIME = "race_start";
			constexpr std::string_view CN_RACE_END_TIME = "race_end";
			constexpr std::string_view CN_EVENING_CHARGING_START_TIME = "eod_charge_start";
			constexpr std::string_view CN_EVENING_CHARGING_END_TIME = "eod_charge_end";
		}  // namespace schedule_day

		namespace checkpoint_info {
			constexpr std::string_view CN_CHECKPOINT_OPEN_TIME = "open_time";
			constexpr std::string_view CN_CHECKPOINT_CLOSE_TIME = "close_time";
			constexpr std::string_view CN_CHECKPOINT_EARLIEST_RELEASE_TIME = "earliest_release_time";
			constexpr std::string_view CN_LOOP_CLOSE_TIME = "loop_close_time";
		}  // namespace checkpoint_info
	}  // namespace race_schedule
	namespace route {
		constexpr int NUM_COLUMNS_ROUTE_FILE = 15;

		constexpr std::string_view CN_START_LATITUDE = "start_latitude";
		constexpr std::string_view CN_START_LONGITUDE = "start_longitude";
		constexpr std::string_view CN_END_LATITUDE = "end_latitude";
		constexpr std::string_view CN_END_LONGITUDE = "end_longitude";
		constexpr std::string_view CN_SEGMENT_END_CONDITION = "segment_end_condition";
		constexpr std::string_view CN_SEGMENT_TYPE = "segment_type";
		constexpr std::string_view CN_SPEED_LIMIT = "speed_limit";
		constexpr std::string_view CN_DISTANCE = "distance";
		constexpr std::string_view CN_HEADING = "heading";
		constexpr std::string_view CN_ELEVATION = "elevation";
		constexpr std::string_view CN_GRADE = "grade";
		constexpr std::string_view CN_ROAD_INCLINE_ANGLE = "road_incline_angle";
		constexpr std::string_view CN_SINE_ROAD_INCLINE_ANGLE = "sine_road_incline_angle";
		constexpr std::string_view CN_GRAVITY = "gravity";
		constexpr std::string_view CN_GRAVITY_TIMES_SINE_ROAD_INCLINE_ANGLE = "gravity_times_sine_road_incline_angle";

		// column order (For interpolation)
		/*
		Weather station, 1
		heading, 2
		speed_limit, 3
		elevation, 4
		grade, 5
		road_incline_angle, 6
		sine_road_incline_angle, 7
		gravity, 8
		gravity_times_sine_road_angle, 9
		*/
		static const int INTERPOLATE_WEATHER_STATION = 0;
		static const int INTERPOLATE_HEADING = 1;
		static const int INTERPOLATE_SPEED_LIMIT = 2;
		static const int INTERPOLATE_ELEVATION = 3;
		static const int INTERPOLATE_GRADE = 4;
		static const int INTERPOLATE_ROAD_INCLINE_ANGLE = 5;
		static const int INTERPOLATE_SINE_ROAD_INCLINE_ANGLE = 6;
		static const int INTERPOLATE_GRAVITY = 7;
		static const int INTERPOLATE_GRAVITY_TIMES_SINE_ROAD_ANGLE = 8;

		namespace end_condition {
			constexpr std::string_view CONTROL_STOP = "control_stop";
			constexpr std::string_view END_OF_RACE = "endOfRace";
			constexpr std::string_view FINISH_LINE = "finishLine";
			constexpr std::string_view MAX_CURVATURE_REACHED = "max_heading_change";
			constexpr std::string_view MAX_GRADE_CHANGE_REACHED = "max_grade_change";
			constexpr std::string_view MAX_LENGTH_REACHED = "max_length";
			constexpr std::string_view SPEED_LIMIT_CHANGE = "speed_limit_change";
			constexpr std::string_view TRAFFIC_LIGHT = "traffic_light";
			constexpr std::string_view YIELD_SIGN_OR_BLINKING_YELLOW = "yield_sign_or_blinking_yellow";
			constexpr std::string_view NULL_CONDITION = "";  // NOLINT
		}  // namespace end_condition

		namespace segment_type {
			constexpr std::string_view RACE = "race";
			constexpr std::string_view MARSHALING = "marshaling";
		}  // namespace segment_type

		namespace clustering {
			static constexpr double maximum_grade_delta = 0.01;
			static constexpr double maximum_heading_delta = deg_to_rad(5);
			static constexpr int maximum_cluster_length_meters = 50000;
		}  // namespace clustering
	}  // namespace route
	namespace weather {
		static constexpr unsigned long WEATHER_FILE_NUMBER_OF_COLUMNS = 10;
		static constexpr unsigned long SOLAR_FILE_NUMBER_OF_COLUMNS = 5;
		static constexpr unsigned long DIM_OF_WEATHER_INTERP = 6;
		static constexpr unsigned long DIM_OF_SOLAR_INTERP = 3;
		// column names
		constexpr std::string_view CN_WEATHER_STATION = "weather_group";
		constexpr std::string_view CN_UNIX_PERIOD = "period_time_unix";
		constexpr std::string_view CN_AIR_TEMPERATURE_2M = "air_temp_2m";
		constexpr std::string_view CN_DHI = "dhi";
		constexpr std::string_view CN_DNI = "dni";
		constexpr std::string_view CN_GHI = "ghi";
		constexpr std::string_view CN_WIND_VELOCITY_NS = "wind_velocity_10m_ns";
		constexpr std::string_view CN_WIND_VELOCITY_EW = "wind_velocity_10m_ew";
		constexpr std::string_view CN_AIR_DENSITY = "air_density";
		constexpr std::string_view CN_SURFACE_PRESSURE = "surface_pressure";
		// column order
		static const int CO_DHI = 0;
		static const int CO_DNI = 1;
		static const int CO_GHI = 2;
		static const int CO_WIND_VELOCITY_NS = 3;
		static const int CO_WIND_VELOCITY_EW = 4;
		static const int CO_AIR_TEMPERATURE_2M = 5;
		static const int CO_SURFACE_PRESSURE = 6;
		static const int CO_AIR_DENSITY = 7;
		namespace stations {
			constexpr int NUM_COLUMNS_STATIONS_FILE = 4;
			static constexpr std::string_view CN_STATION_NAME = "city";
			static constexpr std::string_view CN_STATION_ID = "group";
			static constexpr std::string_view CN_STATION_LATITUDE = "lat";
			static constexpr std::string_view CN_STATION_LONGITUDE = "lon";

		}  // namespace stations
	}  // namespace weather

}  // namespace race_config

#endif  // MINISIM_RACECONFIGCONSTANTS_H