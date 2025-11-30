add_library(cxxopts INTERFACE)

target_include_directories(
	cxxopts
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
		$<INSTALL_INTERFACE:include>
)

set_target_properties(
	cxxopts
	PROPERTIES
		LINKER_LANGUAGE
			CXX
)

target_include_directories(cxxopts INTERFACE ${PROJECT_SOURCE_DIR}/external)

# Compile with no warnings

# target_compile_options(cxxopts PRIVATE -w)
