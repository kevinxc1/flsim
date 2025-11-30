#ifndef MINISIM_WEATHERSTATIONCONSTANTS_H
#define MINISIM_WEATHERSTATIONCONSTANTS_H

#include <string_view>

namespace weather::stations {
	constexpr int NUM_COLUMNS_STATIONS_FILE = 4;
	static constexpr std::string_view CN_STATION_NAME = "city";
	static constexpr std::string_view CN_STATION_ID = "group";
	static constexpr std::string_view CN_STATION_LATITUDE = "lat";
	static constexpr std::string_view CN_STATION_LONGITUDE = "lon";

}  // namespace weather::stations

#endif  // MINISIM_WEATHERSTATIONCONSTANTS_H
