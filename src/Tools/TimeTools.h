#ifndef MINISIM_TIMETOOLS_H
#define MINISIM_TIMETOOLS_H
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

#include "date/date.h"

int parse_time(const std::string s);

struct SplitTime final {
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
	int nanosecond;
};

SplitTime split_time(double t);

SplitTime split_time(time_t t);

std::string format_time_for_file(SplitTime s);

#endif