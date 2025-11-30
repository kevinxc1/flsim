#ifndef MINISIM_ROUTECONSTANTS_H
#define MINISIM_ROUTECONSTANTS_H

#include <string>

#include "Tools/Conversions.h"

namespace route {
	constexpr int NUM_COLUMNS_ROUTE_FILE = 16;

	static const std::string CN_START_LATITUDE = "start_latitude";
	static const std::string CN_START_LONGITUDE = "start_longitude";
	static const std::string CN_END_LATITUDE = "end_latitude";
	static const std::string CN_END_LONGITUDE = "end_longitude";
	static const std::string CN_SEGMENT_END_CONDITION = "segment_end_condition";
	static const std::string CN_SEGMENT_TYPE = "segment_type";
	static const std::string CN_SPEED_LIMIT = "speed_limit";
	static const std::string CN_WEATHER_STATION_INDEX = "weather_group";
	static const std::string CN_DISTANCE = "distance";
	static const std::string CN_HEADING = "heading";
	static const std::string CN_ELEVATION = "elevation";
	static const std::string CN_GRADE = "grade";
	static const std::string CN_ROAD_INCLINE_ANGLE = "road_incline_angle";
	static const std::string CN_SINE_ROAD_INCLINE_ANGLE = "sine_road_incline_angle";
	static const std::string CN_GRAVITY = "gravity";
	static const std::string CN_GRAVITY_TIMES_SINE_ROAD_ANGLE = "gravity_times_sine_road_incline_angle";

	namespace end_condition {
		static const std::string CONTROL_STOP = "control_stop";
		static const std::string END_OF_RACE = "endOfRace";
		static const std::string FINISH_LINE = "finishLine";
		static const std::string MAX_CURVATURE_REACHED = "max_heading_change";
		static const std::string MAX_GRADE_CHANGE_REACHED = "max_grade_change";
		static const std::string MAX_LENGTH_REACHED = "max_length";
		static const std::string SPEED_LIMIT_CHANGE = "speed_limit_change";
		static const std::string TRAFFIC_LIGHT = "traffic_light";
		static const std::string YIELD_SIGN_OR_BLINKING_YELLOW = "yield_sign_or_blinking_yellow";
	}  // namespace end_condition

	namespace segment_type {
		static const std::string RACE = "race";
		static const std::string MARSHALING = "marshaling";
	}  // namespace segment_type

	namespace clustering {
		static constexpr double maximum_grade_delta = 0.01;
		static constexpr double maximum_heading_delta = deg_to_rad(5);
		static constexpr int maximum_cluster_length_meters = 50000;
	}  // namespace clustering
}  // namespace route

#endif  // MINISIM_ROUTECONSTANTS_H
