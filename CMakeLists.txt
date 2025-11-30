# To understand what this file does, look at the CMake documentation
cmake_minimum_required(VERSION 3.20 FATAL_ERROR)

project(
	minisim
	VERSION 0.1
	DESCRIPTION "A Mini Simulator for Strategy Training"
	LANGUAGES
		CXX
)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# TODO Add more compiler flags later
set(CMAKE_CXX_FLAGS "-Wall -Wextra -Wpedantic -Werror -Wno-error=attributes")
if(CMAKE_BUILD_TYPE STREQUAL "Release")
	# Compiler Flags
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O3")

	# LTO / IPO
	include(CheckIPOSupported)
	check_ipo_supported(RESULT ipo_supported OUTPUT error)
	option(ENABLE_LTO "Enable cross language linking time optimization" ON)
	if(ipo_supported)
		message(STATUS "IPO / LTO enabled")
		set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
	else()
		message(STATUS "IPO / LTO not supported: <${error}>")
	endif()

	# Turn Off Assertions
	add_compile_definitions(NDEBUG)
elseif(CMAKE_BUILD_TYPE STREQUAL "Debug")
	# DEBUG_MODE and DEBUG Macros
	add_compile_definitions(DEBUG)
	# Compiler Flags
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0")

	# LTO / IPO
	option(ENABLE_LTO "Enable cross language linking time optimization" OFF)
else()
	# LTO / IPO
	option(ENABLE_LTO "Enable cross language linking time optimization" OFF)
endif()

set(BUILD_SHARED_LIBS OFF)

# Print all the compiler flags (so we know how the compiler is being configured)
message("-- C++ compiler flags: ${CMAKE_CXX_FLAGS}")

include_directories(${CMAKE_BINARY_DIR})
include_directories(${PROJECT_SOURCE_DIR})

add_subdirectory(external)

# Package Manager
include(cmake/CPM.cmake)

# Find packages go here.
include(cmake/catch2.cmake) # Testing
include(CTest)
include(Catch)

include_directories("${PROJECT_SOURCE_DIR}/src")
include_directories("${PROJECT_SOURCE_DIR}/external")
add_subdirectory(src)
