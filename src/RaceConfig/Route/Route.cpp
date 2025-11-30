#include "Route.h"

#include <numeric>
#include <ranges>
#include <span>
#include <string>
#include <vector>

#include "RouteConstants.h"
#include "Tools/Conversions.h"
#include "csv/csv.h"

Route::Route(std::string_view route_file, WeatherStations weather_stations)
	: weather_stations(std::move(weather_stations)) {
	io::CSVReader<route::NUM_COLUMNS_ROUTE_FILE> route_csv(route_file.data());

	route_csv.read_header(io::ignore_extra_column,
		route::CN_START_LATITUDE,                 
		route::CN_START_LONGITUDE,                
		route::CN_END_LATITUDE,                   
		route::CN_END_LONGITUDE,                  
		route::CN_SEGMENT_END_CONDITION,          
		route::CN_SEGMENT_TYPE,                   
		route::CN_SPEED_LIMIT,                    
		route::CN_WEATHER_STATION_INDEX,          
		route::CN_DISTANCE,                       
		route::CN_HEADING,                        
		route::CN_ELEVATION,                      
		route::CN_GRADE,                          
		route::CN_ROAD_INCLINE_ANGLE,             
		route::CN_SINE_ROAD_INCLINE_ANGLE,        
		route::CN_GRAVITY,                        
		route::CN_GRAVITY_TIMES_SINE_ROAD_ANGLE   
	);

	struct RouteSegmentData {
		double weather_station;
		GeographicalCoordinate start_coordinate;
		GeographicalCoordinate end_coordinate;
		std::string end_condition;
		std::string type;
		double speed_limit;
		double distance;
		double heading;
		double elevation;
		double grade;
		double road_incline_angle;
		double sine_road_incline_angle;
		double gravity;
		double gravity_times_sine_road_incline_angle;
	};

	std::vector<RouteSegmentData> route_segment_data;

	while (true) {
		RouteSegmentData current_segment_data;

		if (!route_csv.read_row(current_segment_data.start_coordinate.latitude,   
				current_segment_data.start_coordinate.longitude,                  
				current_segment_data.end_coordinate.latitude,                     
				current_segment_data.end_coordinate.longitude,                    
				current_segment_data.end_condition,                               
				current_segment_data.type,                                        
				current_segment_data.speed_limit,                                 
				current_segment_data.weather_station,                             
				current_segment_data.distance,                                    
				current_segment_data.heading,                                     
				current_segment_data.elevation,                                   
				current_segment_data.grade,                                       
				current_segment_data.road_incline_angle,                          
				current_segment_data.sine_road_incline_angle,                     
				current_segment_data.gravity,                                     
				current_segment_data.gravity_times_sine_road_incline_angle        
				)) {
			break;
		}
		route_segment_data.push_back(current_segment_data);
	}

	for (const auto& segment_data : route_segment_data) {
		const RouteSegment segment{
			.coordinate_start = segment_data.start_coordinate,
			.coordinate_end = segment_data.end_coordinate,
			.end_condition = parse_segment_end_condition(segment_data.end_condition),
			.type = parse_segment_type(segment_data.type),
			.speed_limit = segment_data.speed_limit,
			.weather_station = segment_data.weather_station,
			.distance = segment_data.distance,
			.heading = segment_data.heading,
			.elevation = segment_data.elevation,
			.grade = segment_data.grade,
			.road_incline_angle = segment_data.road_incline_angle,
			.sine_road_incline_angle = segment_data.sine_road_incline_angle,
			.gravity = segment_data.gravity,
			.gravity_times_sine_road_incline_angle = segment_data.gravity_times_sine_road_incline_angle,
		};

		total_distance += segment.distance;
		segments.push_back(segment);
	}
}

RouteSegment Route::get_segment(size_t index) const {
	return segments[index];
}

RouteSegment Route::operator[](size_t index) const {
	return segments[index];
}

const std::vector<RouteSegment>* Route::get_segments() const {
	return &segments;
}

std::span<const RouteSegment> Route::get_segments_span() const {
	return segments;
}

size_t Route::get_num_segments() const {
	return segments.size();
}

size_t Route::get_num_weather_stations() const {
	return weather_stations.size();
}

double Route::get_total_distance() const {
	return total_distance;
}

double Route::get_distance_between(size_t index1, size_t index2) const {
	if (index1 == index2) {
		return 0;
	}
	if (index1 > index2) {
		std::swap(index1, index2);
	}
	if (index1 > segments.size() || index2 > segments.size()) {
		return 0;
	}
	const auto range = std::ranges::views::iota(index1, index2);
	return std::accumulate(
		range.begin(), range.end(), 0.0, [this](double sum, size_t index) { return sum + segments[index].distance; });
}
