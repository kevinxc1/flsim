#ifndef MINISIM_CONVERSIONS_H
#define MINISIM_CONVERSIONS_H

#include <cmath>
#include <numbers>

/// Converts radians to degrees
inline constexpr double rad_to_deg(double angle_in_radians) {
	constexpr double RAD_TO_DEG_FACTOR = 180.0L / std::numbers::pi;
	return angle_in_radians * RAD_TO_DEG_FACTOR;
}

/// Converts degrees to radians
inline constexpr double deg_to_rad(double angle_in_degrees) {
	constexpr double DEG_TO_RAD_FACTOR = std::numbers::pi / 180.0L;
	return angle_in_degrees * DEG_TO_RAD_FACTOR;
}

/// Converts rad/s to RPM
inline constexpr double rad_per_sec_to_rpm(double rad_per_sec) {
	constexpr double SECONDS_PER_MINUTE = 60.0;
	return rad_per_sec * SECONDS_PER_MINUTE / (2 * std::numbers::pi);
}

/// Converts RPM to rad/s
inline constexpr double rpm_to_rad_per_sec(double rpm) {
	constexpr double SECONDS_PER_MINUTE = 60.0;
	return rpm * (2 * std::numbers::pi) / SECONDS_PER_MINUTE;
}

/// Converts RPM and torque to power (w)
inline constexpr double get_power(double rpm, double torque) {
	return rpm_to_rad_per_sec(rpm) * torque;
}

/// Convert a decimal to a percentage
inline constexpr double decimal_to_percent(double decimal) {
	return decimal * 100.0;
}

/// Convert Kilometers to Meters
inline constexpr double kilometer_to_meter(double kilometer) {
	constexpr double METERS_PER_KILOMETER = 1000.0;
	return kilometer * METERS_PER_KILOMETER;
}

/// Convert Kilometers to Meters
inline constexpr double meter_to_kilometer(double meter) {
	constexpr double KILOMETERS_PER_METER = 0.001;
	return meter * KILOMETERS_PER_METER;
}

/// Convert Miles to Meters
inline constexpr double mile_to_meter(double mile) {
	constexpr double METERS_PER_MILE = 1609.344;
	return mile * METERS_PER_MILE;
}

/// Convert Meters to Miles
inline constexpr double meter_to_mile(double meter) {
	constexpr double MILES_PER_METER = 0.000621371;
	return meter * MILES_PER_METER;
}

/// Convert Pascals to Millibars
inline constexpr double pascal_to_millibar(double pascals) {
	constexpr double MILLIBARS_PER_PASCAL = 0.01;
	return pascals * MILLIBARS_PER_PASCAL;
}

/// Convert Grade (Decimal) to Radians
inline constexpr double grade_to_rad(double grade) {
	return std::atan(grade);
}

/// Convert seconds to hours
inline constexpr double seconds_to_hours(double seconds) {
	constexpr double SECONDS_PER_HOUR = 3600.0;
	return seconds / SECONDS_PER_HOUR;
}

/// Convert hours to days
inline constexpr double hours_to_days(double hours) {
	constexpr double HOURS_PER_DAY = 24.0;
	return hours / HOURS_PER_DAY;
}

/// Convert seconds to days
inline constexpr double seconds_to_days(double seconds) {
	return hours_to_days(seconds_to_hours(seconds));
}

/// Convert days to hours
inline constexpr double days_to_hours(double days) {
	constexpr double HOURS_PER_DAY = 24.0;
	return days * HOURS_PER_DAY;
}

/// Convert hours to seconds
inline constexpr double hours_to_seconds(double hours) {
	constexpr double SECONDS_PER_HOUR = 3600.0;
	return hours * SECONDS_PER_HOUR;
}

/// Convert days to seconds
inline constexpr double days_to_seconds(double days) {
	return hours_to_seconds(days_to_hours(days));
}

/// Converts m/s to kph
inline constexpr double mps_to_kph(const double meters_per_second) {
	constexpr double MPS_TO_KPH_FACTOR = 3.6;
	return meters_per_second * MPS_TO_KPH_FACTOR;
}

/// Watt-hours to Joules
inline constexpr double watt_hours_to_joules(const double watt_hours) {
	constexpr double WATT_HOURS_TO_JOULES_FACTOR = 3600.0;
	return watt_hours * WATT_HOURS_TO_JOULES_FACTOR;
}

/// Celsius to Kelvin
inline constexpr double celsius_to_kelvin(const double celsius) {
	constexpr double CELSIUS_TO_KELVIN_FACTOR = 273.15;
	return celsius + CELSIUS_TO_KELVIN_FACTOR;
}

/// Kelvin to Celsius
inline constexpr double kelvin_to_celsius(const double kelvin) {
	constexpr double KELVIN_TO_CELSIUS_FACTOR = 273.15;
	return kelvin - KELVIN_TO_CELSIUS_FACTOR;
}

#endif  // MINISIM_CONVERSIONS_H
