add_library(optimizers "")

target_sources(
	optimizers
	PUBLIC
		Optimizer.h
		BinarySearchOptimizer.h
		LinearSearchOptimizer.h
	PRIVATE
		Optimizer.cpp
		BinarySearchOptimizer.cpp
		LinearSearchOptimizer.cpp
)

target_link_libraries(optimizers PUBLIC raceconfig PRIVATE racerunner)

target_include_directories(optimizers PUBLIC ${PROJECT_SOURCE_DIR}/src)
