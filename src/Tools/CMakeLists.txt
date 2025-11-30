add_library(conversions INTERFACE Conversions.h)
target_include_directories(conversions INTERFACE ${PROJECT_SOURCE_DIR}/src)

add_library(physical_constants INTERFACE PhysicalConstants.h)
target_include_directories(
	physical_constants
	INTERFACE
		${PROJECT_SOURCE_DIR}/src
)

# Sets a constant string value in Tools/RootDirectory.h ROOT_DIRECTORY becomes the users root
# directory to allow easy File IO
set(ROOT_DIR ${CMAKE_SOURCE_DIR})
configure_file(RootDirectory.h.in RootDirectory.h @ONLY)

add_library(root_tool "")
target_sources(
	root_tool
	PRIVATE
		RootDirectory.cpp
	PUBLIC
		${CMAKE_BINARY_DIR}/src/Tools/RootDirectory.h
)
target_include_directories(root_tool PUBLIC ${CMAKE_BINARY_DIR}/src)

add_library(file_tools "")
target_sources(file_tools PRIVATE FileTools.cpp PUBLIC FileTools.h)
target_link_libraries(file_tools PRIVATE root_tool)

add_library(root_binary_search "")
target_sources(
	root_binary_search
	PRIVATE
		RootBinarySearch.cpp
	PUBLIC
		RootBinarySearch.h
)

add_library(parsing "")
target_sources(parsing PRIVATE Parsing.cpp PUBLIC Parsing.h)
target_link_libraries(
	parsing
	PRIVATE
		alglib
		external_tools
)
target_include_directories(parsing INTERFACE ${PROJECT_SOURCE_DIR}/src)

add_library(time_tools "")
target_sources(time_tools PRIVATE TimeTools.cpp PUBLIC TimeTools.h)
target_link_libraries(
	time_tools
	PRIVATE
		date
		external_tools
)
target_include_directories(time_tools INTERFACE ${PROJECT_SOURCE_DIR}/src)

add_library(internal_tools INTERFACE)
target_link_libraries(
	internal_tools
	INTERFACE
		conversions
		parsing
		physical_constants
		root_tool
		file_tools
		time_tools
)
target_include_directories(internal_tools INTERFACE ${PROJECT_SOURCE_DIR}/src)
