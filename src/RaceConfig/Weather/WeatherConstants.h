#ifndef MINISIM_WEATHERCONSTANTS_H
#define MINISIM_WEATHERCONSTANTS_H

#include <string>

namespace weather {
	static constexpr unsigned long WEATHER_FILE_NUMBER_OF_COLUMNS = 11;
	// column names
	static const std::string CN_WEATHER_STATION = "weather_group";
	static const std::string CN_UNIX_PERIOD = "period_time_unix";
	static const std::string CN_AIR_TEMPERATURE_2M = "air_temp_2m";
	static const std::string CN_DHI = "dhi";
	static const std::string CN_DNI = "dni";
	static const std::string CN_GHI = "ghi";
	static const std::string CN_WIND_VELOCITY_NS = "wind_velocity_10m_ns";
	static const std::string CN_WIND_VELOCITY_EW = "wind_velocity_10m_ew";
	static const std::string CN_AIR_DENSITY = "air_density";
	static const std::string CN_SURFACE_PRESSURE = "surface_pressure";
	static const std::string CN_RECIPROCAL_SPEED_OF_SOUND = "reciprocal_speed_of_sound";
	// column order
	static const int CO_DHI = 0;
	static const int CO_DNI = 1;
	static const int CO_GHI = 2;
	static const int CO_WIND_VELOCITY_NS = 3;
	static const int CO_WIND_VELOCITY_EW = 4;
	static const int CO_AIR_TEMPERATURE_2M = 5;
	static const int CO_SURFACE_PRESSURE = 6;
	static const int CO_AIR_DENSITY = 7;
	static const int CO_RECIPROCAL_SPEED_OF_SOUND = 8;
}  // namespace weather

#endif  // MINISIM_WEATHERCONSTANTS_H