#ifndef MINISIM_SOLARPOSITION_H
#define MINISIM_SOLARPOSITION_H

#include <optional>

#include "DataClasses/GeographicalCoordinate.h"

/// A Wrapper to abstract away calculating Solar Position using NREL's
/// Solar Position Algorithm Library
namespace SolarPosition {
	/// unnamed namespace to make it private / hold constants
	namespace internal {
		/// We only need azimuth, zenith, and incidence angle, so we will only calculate those

		/// Delta UT1 is the difference between UTC and UT from http://maia.usno.navy.mil/ser7/ser7.dat
		/// This value is now 0.0, but it is kept here in case it is needed in the future
		///
		/// Units: seconds
		constexpr double DELTA_UT1 = 0.0;

		/// TAI - UTC is the difference between TAI and UTC from http://maia.usno.navy.mil/ser7/ser7.dat
		/// Reference the link intermittently to make sure this value is correct, at the moment of creation it
		/// was 37.000000.
		///
		/// Note: this value is slightly different depending on the year, so old data may not be as precise / accurate
		/// to higher precision. However, the difference is small enough that it should not be a problem.
		///
		/// Units: seconds
		constexpr double TAI_MINUS_UTC = 37.000'000;

		/// Delta T is the difference between earth rotation time and terrestrial time.
		/// Again, refer to http://maia.usno.navy.mil/ser7/ser7.dat and use the formula
		/// Delta T = 32.184 + (TAI-UTC) - DUT1
		///
		/// Units: seconds
		constexpr double DELTA_T = 32.184 + TAI_MINUS_UTC - DELTA_UT1;

		/// The Atmospheric Refraction Index can make a difference in apparent solar position, so the algorithm
		/// allows for it to be set. However, for the purposes of the simulator, we will assume a value of 0.5667
		/// degrees which is the typical value cited by the NREL SPA documentation. However, this is a point to consider
		/// when making minute changes to the simulator (though the error might be a factor of very few arcminutes).
		///
		/// Units: degrees
		constexpr double ATMOSPHERIC_REFRACTION_INDEX = 0.5667;

		/// There doesn't seem to be much information online about the Surface Azimuth Rotation Angle, but it is
		/// something that seems to impact the solar position calculation minutely (it shifts the `topocentric
		/// astronomers azimuth angle`). For now, we'll assume a value of 0.0, but this is something that will need to
		/// be referenced / fixed.
		///
		/// Units: degrees
		// TODO: Find out what this value should be
		constexpr double SURFACE_AZIMUTH_ROTATION_ANGLE = 0.0;

		/// For all purposes of the simulator, the timezone we will be using is UTC.
		/// Therefore, we are assuming a timezone offset of 0.
		///
		/// Units: hours
		constexpr double TIMEZONE_OFFSET = 0.0;
	}  // namespace internal

	struct SolarPositionConfig {
		double pressure;     // millibars
		double temperature;  // Celsius
		double elevation;
		double timestamp;
		GeographicalCoordinate coordinate;
	};

	struct SolarPositionData {
		/// Radians Azimuth of the Sun
		double azimuth;
		/// Radians Zenith of the Sun
		double zenith;
	};

	namespace solpos {
		SolarPositionData calculate(const SolarPositionConfig& config);
	}

}  // namespace SolarPosition

#endif  // MINISIM_SOLARPOSITION_H