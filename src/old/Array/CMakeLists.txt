add_library(array STATIC)

target_sources(array PUBLIC Array.h PRIVATE Array.cpp)

# add_executable(array_test_gen array_test_gen.cpp)
# target_link_libraries(array_test_gen PUBLIC array)

add_executable(array_tests ArrayTests.cpp)
target_link_libraries(
	array_tests
	PRIVATE
		array
		Catch2::Catch2WithMain
)

catch_discover_tests(array_tests)
