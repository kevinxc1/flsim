add_subdirectory(alglib)
add_subdirectory(csv)
add_subdirectory(cxxopts)
add_subdirectory(date)
add_subdirectory(solpos)
add_subdirectory(toml)

add_library(external_tools INTERFACE)

target_link_libraries(
	external_tools
	INTERFACE
		alglib
		csv
		date
		solpos
		cxxopts
		tomlplusplus
)

target_include_directories(
	external_tools
	INTERFACE
		${PROJECT_SOURCE_DIR}/external
)
