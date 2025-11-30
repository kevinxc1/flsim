add_subdirectory(ConfigFile)
add_subdirectory(DataClasses)
add_subdirectory(RaceConfig)
add_subdirectory(SolarCar)
add_subdirectory(Tools)
add_subdirectory(RaceSegmentRunner)
add_subdirectory(RaceRunner)
add_subdirectory(Optimizer)

add_library(tools INTERFACE)
target_link_libraries(
	tools
	INTERFACE
		config_file
		dataclasses
		external_tools
		internal_tools
)

target_include_directories(
	tools
	INTERFACE
		${PROJECT_SOURCE_DIR}/src
		${PROJECT_SOURCE_DIR}/external
)

add_library(simulator_dependencies INTERFACE)
target_link_libraries(
	simulator_dependencies
	INTERFACE
		solarcar
		raceconfig
		racerunner
		optimizers
)

add_executable(minisim minisim.cpp)
target_link_libraries(
	minisim
	PRIVATE
		tools
		simulator_dependencies
)

add_custom_target(
	MinisimLink
	ALL
	COMMAND
		${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_BINARY_DIR}/minisim
		${CMAKE_BINARY_DIR}/minisim
	COMMAND
		${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_BINARY_DIR}/minisim
		${CMAKE_BINARY_DIR}/minisim.exe
	DEPENDS
		minisim
	VERBATIM
)
