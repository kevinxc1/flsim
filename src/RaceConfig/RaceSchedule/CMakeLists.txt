add_library(raceschedule "")

target_sources(
	raceschedule
	PRIVATE
		RaceSchedule.cpp
	PUBLIC
		RaceSchedule.h
		RaceScheduleConstants.h
)

target_link_libraries(raceschedule PRIVATE tools)
