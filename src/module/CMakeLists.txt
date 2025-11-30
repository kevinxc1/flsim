add_subdirectory(Aerobody)
add_subdirectory(Array)
add_subdirectory(Battery)
add_subdirectory(Motor)
add_subdirectory(Tire)

add_library(solarcar "")
target_sources(solarcar PRIVATE SolarCar.cpp PUBLIC SolarCar.h)
target_link_libraries(
	solarcar
	PUBLIC
		config_file
		aerobody
		array
		battery
		motor
		tire
)
