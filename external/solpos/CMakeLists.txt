add_library(solpos "")

# NOTE: solpos.c was renamed solpos.cpp to compile easily with CMake
target_sources(solpos PRIVATE solpos.cpp PUBLIC solpos.h)

target_include_directories(solpos INTERFACE ${PROJECT_SOURCE_DIR}/external)

target_compile_options(
	solpos
	PRIVATE
		-w
		-O3
)
