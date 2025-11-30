#include "TimeTools.h"

namespace {
	 
	date::sys_time<std::chrono::milliseconds> parse(const std::string s) {
		std::istringstream ss(s);
		std::istream& is(ss);
		std::string save;
		is >> save;
		std::istringstream in{save};
		date::sys_time<std::chrono::milliseconds> tp;
		in >> date::parse("%FT%TZ", tp);
		if (in.fail()) {
			in.clear();
			in.exceptions(std::ios::failbit);
			in.str(save);
			in >> date::parse("%FT%T%Ez", tp);
		}
		return tp;
	}

	 
	std::string to_format(const int number) {
		std::stringstream ss;
		ss << std::setw(2) << std::setfill('0') << number;
		return ss.str();
	}
}   

int parse_time(const std::string s) {
	auto time = parse(s);
	auto duration = time.time_since_epoch() / std::chrono::milliseconds(1);

	 
	duration /= 1000;
	int64_t seconds = duration;

	return seconds;
}

 
SplitTime split_time(double time) {
	return split_time(time_t(time));
}

 
SplitTime split_time(time_t t) {
	SplitTime ret;
	 

	tm ltm;               
	gmtime_r(&t, &ltm);   
	 
	ret.year = ltm.tm_year + 1900;
	 
	ret.month = ltm.tm_mon + 1;
	ret.day = ltm.tm_mday;
	ret.hour = ltm.tm_hour;
	ret.minute = ltm.tm_min;
	ret.second = ltm.tm_sec;

	return ret;
}

std::string format_time_for_file(SplitTime s) {
	 
	std::stringstream formatted_time;
	formatted_time << std::to_string(s.year) << "-" << to_format(s.month) << "-" << to_format(s.day) << "_"
				   << to_format(s.hour) << "." << to_format(s.minute) << "." << to_format(s.second);
	return formatted_time.str();
}