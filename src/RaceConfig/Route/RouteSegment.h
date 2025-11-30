#ifndef MINISIM_ROUTESEGMENT_H
#define MINISIM_ROUTESEGMENT_H

#include <string>
#include <unordered_map>

#include "DataClasses/GeographicalCoordinate.h"
#include "RouteConstants.h"

/// The reason that the segment ends
enum SegmentEndCondition {
	CONTROL_STOP,
	END_OF_RACE,
	FINISH_LINE,
	MAX_CURVATURE_REACHED,
	MAX_GRADE_CHANGE_REACHED,
	MAX_LENGTH_REACHED,
	SPEED_LIMIT_CHANGE,
	TRAFFIC_LIGHT,
	YIELD_SIGN_OR_BLINKING_YELLOW
};

/// Segment Types:
///   - RACE: Actual Race Segments
///   - PRACTICE: Segments after the end of the official race
/// We need to make sure that we have enough energy to stay moving throughout marhsaling segments, but they are not as
/// important since the car is moving slowly.
/// HOWEVER, if the weather is bad on marshaling day, we need to ensure we can still move through the race or face
/// sever penalties.
enum SegmentType { RACE, MARSHALING };

static const std::unordered_map<std::string, SegmentEndCondition> string_to_segment_end_condition_map = {
	{route::end_condition::CONTROL_STOP,                  SegmentEndCondition::CONTROL_STOP                 },
	{route::end_condition::END_OF_RACE,                   SegmentEndCondition::END_OF_RACE                  },
	{route::end_condition::FINISH_LINE,                   SegmentEndCondition::FINISH_LINE                  },
	{route::end_condition::MAX_CURVATURE_REACHED,         SegmentEndCondition::MAX_CURVATURE_REACHED        },
	{route::end_condition::MAX_GRADE_CHANGE_REACHED,      SegmentEndCondition::MAX_GRADE_CHANGE_REACHED     },
	{route::end_condition::MAX_LENGTH_REACHED,            SegmentEndCondition::MAX_LENGTH_REACHED           },
	{route::end_condition::SPEED_LIMIT_CHANGE,            SegmentEndCondition::SPEED_LIMIT_CHANGE           },
	{route::end_condition::TRAFFIC_LIGHT,                 SegmentEndCondition::TRAFFIC_LIGHT                },
	{route::end_condition::YIELD_SIGN_OR_BLINKING_YELLOW, SegmentEndCondition::YIELD_SIGN_OR_BLINKING_YELLOW}
};

static const std::unordered_map<std::string, SegmentType> string_to_segment_type_map = {
	{route::segment_type::RACE,       SegmentType::RACE      },
    {route::segment_type::MARSHALING, SegmentType::MARSHALING}
};

[[maybe_unused]] static SegmentEndCondition parse_segment_end_condition(const std::string& end_condition_string) {
	if (string_to_segment_end_condition_map.count(end_condition_string) == 0U) {
		throw std::exception();
	}
	return string_to_segment_end_condition_map.at(end_condition_string);
}

[[maybe_unused]] static SegmentType parse_segment_type(const std::string& segment_type_string) {
	if (string_to_segment_type_map.count(segment_type_string) == 0U) {
		throw std::exception();
	}
	return string_to_segment_type_map.at(segment_type_string);
}

/**
 * A Wrapper Class for Route Segments
 *
 * For more documentation, look at docs/
 */
struct RouteSegment {
	/// The start coordinate of the segment.
	GeographicalCoordinate coordinate_start;
	/// The end coordinate of the segment.
	GeographicalCoordinate coordinate_end;
	/// The reason that the segment ends
	SegmentEndCondition end_condition;
	/// The type of the segment.
	SegmentType type;
	/// The speed limit of the segment in km/h.
	double speed_limit;
	/// The weighted average weather group id
	double weather_station;
	/// The haversine distance of the segment (in kilometers).
	double distance;
	/// The heading of the segment (in radians).
	double heading;
	/// The average elevation of the segment (in meters).
	double elevation;
	/// The grade of the segment out of 1 (100% grade = 1).
	/// Note: A negative grade means the segment is downhill.
	double grade;
	/// The road's angle (in radians).
	double road_incline_angle;
	/// [PRECOMPUTE IN ROUTE] The sine of the road's angle.
	double sine_road_incline_angle;
	/// The acceleration due to gravity at at the segment's coordinates (in m/s^2).
	double gravity;
	/// [PRECOMPUTE IN ROUTE] The product of gravity and sine_road_incline_angle.
	double gravity_times_sine_road_incline_angle;
};

#endif  // MINISIM_ROUTESEGMENT_H
