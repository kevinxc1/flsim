add_library(dataclasses "")

target_sources(
	dataclasses
	PRIVATE
		GeographicalCoordinate.cpp
	PUBLIC
		GeographicalCoordinate.h
)

target_include_directories(dataclasses PUBLIC ${PROJECT_SOURCE_DIR}/src)
