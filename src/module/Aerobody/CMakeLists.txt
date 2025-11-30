add_library(aerobody STATIC)

target_sources(aerobody PUBLIC Aerobody.h PRIVATE Aerobody.cpp)

# add_executable(aerobody_test_gen aerobody_test_gen.cpp)
# target_link_libraries(aerobody_test_gen PUBLIC aerobody)

add_executable(aerobody_tests AerobodyTests.cpp)
target_link_libraries(
	aerobody_tests
	PRIVATE
		aerobody
		Catch2::Catch2WithMain
)

catch_discover_tests(aerobody_tests)
