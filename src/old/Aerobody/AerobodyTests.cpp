#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "Aerobody.h"
#include "VelocityVector.hpp"

constexpr double EPSILON = 0.001; // %

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("VelocityVector Constructors: from_cartesian_components", "[VelocityVector]") {
	SECTION("North-South Construction") {
		// North Construction with different magnitudes
		const VelocityVector v1 = VelocityVector::from_cartesian_components(1, 0);
		REQUIRE_THAT(v1.get_north_south(), WithinRel(1, EPSILON));
		REQUIRE_THAT(v1.get_east_west(), WithinAbs(0, EPSILON));

		const VelocityVector v2 = VelocityVector::from_cartesian_components(10, 0);
		REQUIRE_THAT(v2.get_north_south(), WithinRel(10, EPSILON));
		REQUIRE_THAT(v2.get_east_west(), WithinAbs(0, EPSILON));
		const VelocityVector v3 = VelocityVector::from_cartesian_components(std::numbers::pi, 0);
		REQUIRE_THAT(v3.get_north_south(), WithinRel(std::numbers::pi, EPSILON));
		REQUIRE_THAT(v3.get_east_west(), WithinAbs(0, EPSILON));
		// South Construction with different magnitudes
		const VelocityVector v4 = VelocityVector::from_cartesian_components(-1, 0);
		REQUIRE_THAT(v4.get_north_south(), WithinRel(-1, EPSILON));
		REQUIRE_THAT(v4.get_east_west(), WithinAbs(0, EPSILON));

		const VelocityVector v5 = VelocityVector::from_cartesian_components(-10, 0);
		REQUIRE_THAT(v5.get_north_south(), WithinRel(-10, EPSILON));
		REQUIRE_THAT(v5.get_east_west(), WithinAbs(0, EPSILON));
		const VelocityVector v6 = VelocityVector::from_cartesian_components(-std::numbers::pi, 0);
		REQUIRE_THAT(v6.get_north_south(), WithinRel(-std::numbers::pi, EPSILON));
		REQUIRE_THAT(v6.get_east_west(), WithinAbs(0, EPSILON));
	}

	SECTION("East-West Construction") {
		// East Construction with different magnitudes
		const VelocityVector v1 = VelocityVector::from_cartesian_components(0, 1);
		REQUIRE_THAT(v1.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v1.get_east_west(), WithinRel(1, EPSILON));

		const VelocityVector v2 = VelocityVector::from_cartesian_components(0, 10);
		REQUIRE_THAT(v2.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v2.get_east_west(), WithinRel(10, EPSILON));
		const VelocityVector v3 = VelocityVector::from_cartesian_components(0, std::numbers::pi);
		REQUIRE_THAT(v3.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v3.get_east_west(), WithinRel(std::numbers::pi, EPSILON));
		// West Construction with different magnitudes
		const VelocityVector v4 = VelocityVector::from_cartesian_components(0, -1);
		REQUIRE_THAT(v4.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v4.get_east_west(), WithinRel(-1, EPSILON));

		const VelocityVector v5 = VelocityVector::from_cartesian_components(0, -10);
		REQUIRE_THAT(v5.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v5.get_east_west(), WithinRel(-10, EPSILON));
		const VelocityVector v6 = VelocityVector::from_cartesian_components(0, -std::numbers::pi);
		REQUIRE_THAT(v6.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v6.get_east_west(), WithinRel(-std::numbers::pi, EPSILON));
	}

	SECTION("Random Test 0") {
		const VelocityVector v = VelocityVector::from_cartesian_components(-46.4741, -54.1429);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-46.4741, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-54.1429, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector v = VelocityVector::from_cartesian_components(-33.7781, 9.44001);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-33.7781, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(9.44001, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector v = VelocityVector::from_cartesian_components(-26.5379, 77.9927);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-26.5379, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(77.9927, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector v = VelocityVector::from_cartesian_components(25.8619, 31.2886);
		REQUIRE_THAT(v.get_north_south(), WithinRel(25.8619, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(31.2886, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector v = VelocityVector::from_cartesian_components(17.9359, -30.553);
		REQUIRE_THAT(v.get_north_south(), WithinRel(17.9359, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-30.553, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector v = VelocityVector::from_cartesian_components(-14.0983, 21.8987);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-14.0983, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(21.8987, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector v = VelocityVector::from_cartesian_components(-82.0913, -64.2085);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-82.0913, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-64.2085, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector v = VelocityVector::from_cartesian_components(94.0367, -84.0134);
		REQUIRE_THAT(v.get_north_south(), WithinRel(94.0367, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-84.0134, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector v = VelocityVector::from_cartesian_components(59.806, -69.5738);
		REQUIRE_THAT(v.get_north_south(), WithinRel(59.806, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-69.5738, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector v = VelocityVector::from_cartesian_components(81.9, 77.7331);
		REQUIRE_THAT(v.get_north_south(), WithinRel(81.9, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(77.7331, EPSILON));
	}

}

TEST_CASE("VelocityVector Constructors: from_polar_components", "[VelocityVector]") {
	SECTION("North-South Construction") {
		// North Construction with different magnitudes
		const VelocityVector v1 = VelocityVector::from_polar_components(1, 0);
		REQUIRE_THAT(v1.get_north_south(), WithinRel(1, EPSILON));
		REQUIRE_THAT(v1.get_east_west(), WithinAbs(0, EPSILON));

		const VelocityVector v2 = VelocityVector::from_polar_components(10, 0);
		REQUIRE_THAT(v2.get_north_south(), WithinRel(10, EPSILON));
		REQUIRE_THAT(v2.get_east_west(), WithinAbs(0, EPSILON));
		const VelocityVector v3 = VelocityVector::from_polar_components(std::numbers::pi, 0);
		REQUIRE_THAT(v3.get_north_south(), WithinRel(std::numbers::pi, EPSILON));
		REQUIRE_THAT(v3.get_east_west(), WithinAbs(0, EPSILON));
		// South Construction with different magnitudes
		const VelocityVector v4 = VelocityVector::from_polar_components(1, std::numbers::pi);
		REQUIRE_THAT(v4.get_north_south(), WithinRel(-1, EPSILON));
		REQUIRE_THAT(v4.get_east_west(), WithinAbs(0, EPSILON));

		const VelocityVector v5 = VelocityVector::from_polar_components(10, std::numbers::pi);
		REQUIRE_THAT(v5.get_north_south(), WithinRel(-10, EPSILON));
		REQUIRE_THAT(v5.get_east_west(), WithinAbs(0, EPSILON));
		const VelocityVector v6 = VelocityVector::from_polar_components(std::numbers::pi, std::numbers::pi);
		REQUIRE_THAT(v6.get_north_south(), WithinRel(-std::numbers::pi, EPSILON));
		REQUIRE_THAT(v6.get_east_west(), WithinAbs(0, EPSILON));
	}

	SECTION("East-West Construction") {
		// East Construction with different magnitudes
		const VelocityVector v1 = VelocityVector::from_polar_components(1, std::numbers::pi / 2);
		REQUIRE_THAT(v1.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v1.get_east_west(), WithinRel(1, EPSILON));

		const VelocityVector v2 = VelocityVector::from_polar_components(10, std::numbers::pi / 2);
		REQUIRE_THAT(v2.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v2.get_east_west(), WithinRel(10, EPSILON));
		const VelocityVector v3 = VelocityVector::from_polar_components(std::numbers::pi, std::numbers::pi / 2);
		REQUIRE_THAT(v3.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v3.get_east_west(), WithinRel(std::numbers::pi, EPSILON));
		// West Construction with different magnitudes
		const VelocityVector v4 = VelocityVector::from_polar_components(1, -std::numbers::pi / 2);
		REQUIRE_THAT(v4.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v4.get_east_west(), WithinRel(-1, EPSILON));

		const VelocityVector v5 = VelocityVector::from_polar_components(10, -std::numbers::pi / 2);
		REQUIRE_THAT(v5.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v5.get_east_west(), WithinRel(-10, EPSILON));
		const VelocityVector v6 = VelocityVector::from_polar_components(std::numbers::pi, -std::numbers::pi / 2);
		REQUIRE_THAT(v6.get_north_south(), WithinAbs(0, EPSILON));
		REQUIRE_THAT(v6.get_east_west(), WithinRel(-std::numbers::pi, EPSILON));
	}

	SECTION("Random Test 0") {
		const VelocityVector v = VelocityVector::from_polar_components(73.13, -2.00603);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-30.8331, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-66.3123, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector v = VelocityVector::from_polar_components(91.869, -1.5003);
		REQUIRE_THAT(v.get_north_south(), WithinRel(6.47132, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-91.6408, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector v = VelocityVector::from_polar_components(18.2815, -2.63653);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-15.999, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-8.8457, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector v = VelocityVector::from_polar_components(82.6182, 2.21954);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-49.9169, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(65.8337, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector v = VelocityVector::from_polar_components(38.8124, 0.600429);
		REQUIRE_THAT(v.get_north_south(), WithinRel(32.0238, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(21.9289, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector v = VelocityVector::from_polar_components(20.2032, -0.44431);
		REQUIRE_THAT(v.get_north_south(), WithinRel(18.2416, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-8.68404, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector v = VelocityVector::from_polar_components(76.9765, 1.02605);
		REQUIRE_THAT(v.get_north_south(), WithinRel(39.889, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(65.835, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector v = VelocityVector::from_polar_components(5.54899, 2.27457);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-3.59076, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(4.23057, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector v = VelocityVector::from_polar_components(68.4305, -1.10061);
		REQUIRE_THAT(v.get_north_south(), WithinRel(31.0027, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(-61.0046, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector v = VelocityVector::from_polar_components(38.1481, 1.65938);
		REQUIRE_THAT(v.get_north_south(), WithinRel(-3.37506, EPSILON));
		REQUIRE_THAT(v.get_east_west(), WithinRel(37.9985, EPSILON));
	}

}

TEST_CASE("VelocityVector Utilities: get_magnitude", "[VelocityVector]") {
	SECTION("Standard Tests") {
		const VelocityVector v1 = VelocityVector::from_cartesian_components(1, 0);
		REQUIRE_THAT(v1.get_magnitude(), WithinRel(1, EPSILON));
		const VelocityVector v2 = VelocityVector::from_cartesian_components(0, 1);
		REQUIRE_THAT(v2.get_magnitude(), WithinRel(1, EPSILON));
		const VelocityVector v3 = VelocityVector::from_cartesian_components(1, 1);
		REQUIRE_THAT(v3.get_magnitude(), WithinRel(std::sqrt(2), EPSILON));
		const VelocityVector v4 = VelocityVector::from_cartesian_components(3, 4);
		REQUIRE_THAT(v4.get_magnitude(), WithinRel(5, EPSILON));
		const VelocityVector v5 = VelocityVector::from_cartesian_components(5, 12);
		REQUIRE_THAT(v5.get_magnitude(), WithinRel(13, EPSILON));
	}

	SECTION("Random Test 0") {
		const VelocityVector v = VelocityVector::from_polar_components(49.743, 1.46256);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(49.743, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector v = VelocityVector::from_polar_components(51.2064, 3.01844);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(51.2064, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector v = VelocityVector::from_polar_components(96.7518, 0.853592);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(96.7518, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector v = VelocityVector::from_polar_components(98.4532, 1.6417);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(98.4532, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector v = VelocityVector::from_polar_components(98.0927, 2.81412);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(98.0927, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector v = VelocityVector::from_polar_components(12.093, 1.26304);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(12.093, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector v = VelocityVector::from_polar_components(14.8005, 0.571468);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(14.8005, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector v = VelocityVector::from_polar_components(78.0535, 2.49346);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(78.0535, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector v = VelocityVector::from_polar_components(3.32972, 3.00741);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(3.32972, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector v = VelocityVector::from_polar_components(23.3737, 2.5766);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(23.3737, EPSILON));
	}

	SECTION("Random Test 10") {
		const VelocityVector v = VelocityVector::from_polar_components(88.638, -0.12039);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(88.638, EPSILON));
	}

	SECTION("Random Test 11") {
		const VelocityVector v = VelocityVector::from_polar_components(59.6874, 0.138155);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(59.6874, EPSILON));
	}

	SECTION("Random Test 12") {
		const VelocityVector v = VelocityVector::from_polar_components(52.0329, 3.10747);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(52.0329, EPSILON));
	}

	SECTION("Random Test 13") {
		const VelocityVector v = VelocityVector::from_polar_components(87.1476, 1.98529);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(87.1476, EPSILON));
	}

	SECTION("Random Test 14") {
		const VelocityVector v = VelocityVector::from_polar_components(5.39951, 0.532133);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(5.39951, EPSILON));
	}

	SECTION("Random Test 15") {
		const VelocityVector v = VelocityVector::from_polar_components(61.513, -0.352834);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(61.513, EPSILON));
	}

	SECTION("Random Test 16") {
		const VelocityVector v = VelocityVector::from_polar_components(10.4318, 1.14433);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(10.4318, EPSILON));
	}

	SECTION("Random Test 17") {
		const VelocityVector v = VelocityVector::from_polar_components(12.4411, -1.71327);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(12.4411, EPSILON));
	}

	SECTION("Random Test 18") {
		const VelocityVector v = VelocityVector::from_polar_components(54.2229, -2.6654);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(54.2229, EPSILON));
	}

	SECTION("Random Test 19") {
		const VelocityVector v = VelocityVector::from_polar_components(54.4923, -2.62476);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(54.4923, EPSILON));
	}

	SECTION("Random Test 20") {
		const VelocityVector v = VelocityVector::from_polar_components(56.1957, -1.06656);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(56.1957, EPSILON));
	}

	SECTION("Random Test 21") {
		const VelocityVector v = VelocityVector::from_polar_components(47.6422, 1.97908);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(47.6422, EPSILON));
	}

	SECTION("Random Test 22") {
		const VelocityVector v = VelocityVector::from_polar_components(99.2832, 1.66652);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(99.2832, EPSILON));
	}

	SECTION("Random Test 23") {
		const VelocityVector v = VelocityVector::from_polar_components(42.4795, 2.37563);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(42.4795, EPSILON));
	}

	SECTION("Random Test 24") {
		const VelocityVector v = VelocityVector::from_polar_components(51.584, 1.39299);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(51.584, EPSILON));
	}

	SECTION("Random Test 25") {
		const VelocityVector v = VelocityVector::from_polar_components(71.4239, -2.4918);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(71.4239, EPSILON));
	}

	SECTION("Random Test 26") {
		const VelocityVector v = VelocityVector::from_polar_components(71.8805, 2.05116);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(71.8805, EPSILON));
	}

	SECTION("Random Test 27") {
		const VelocityVector v = VelocityVector::from_polar_components(84.2146, -1.99242);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(84.2146, EPSILON));
	}

	SECTION("Random Test 28") {
		const VelocityVector v = VelocityVector::from_polar_components(45.6609, 2.2571);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(45.6609, EPSILON));
	}

	SECTION("Random Test 29") {
		const VelocityVector v = VelocityVector::from_polar_components(33.2448, 0.267094);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(33.2448, EPSILON));
	}

	SECTION("Random Test 30") {
		const VelocityVector v = VelocityVector::from_polar_components(74.522, 1.64198);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(74.522, EPSILON));
	}

	SECTION("Random Test 31") {
		const VelocityVector v = VelocityVector::from_polar_components(7.66458, -2.1743);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(7.66458, EPSILON));
	}

	SECTION("Random Test 32") {
		const VelocityVector v = VelocityVector::from_polar_components(29.6146, -0.685201);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(29.6146, EPSILON));
	}

	SECTION("Random Test 33") {
		const VelocityVector v = VelocityVector::from_polar_components(68.3384, -2.88004);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(68.3384, EPSILON));
	}

	SECTION("Random Test 34") {
		const VelocityVector v = VelocityVector::from_polar_components(77.2992, -2.00037);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(77.2992, EPSILON));
	}

	SECTION("Random Test 35") {
		const VelocityVector v = VelocityVector::from_polar_components(58.1197, 1.75478);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(58.1197, EPSILON));
	}

	SECTION("Random Test 36") {
		const VelocityVector v = VelocityVector::from_polar_components(60.2807, -2.48768);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(60.2807, EPSILON));
	}

	SECTION("Random Test 37") {
		const VelocityVector v = VelocityVector::from_polar_components(88.1591, -3.02592);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(88.1591, EPSILON));
	}

	SECTION("Random Test 38") {
		const VelocityVector v = VelocityVector::from_polar_components(60.2448, 0.845803);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(60.2448, EPSILON));
	}

	SECTION("Random Test 39") {
		const VelocityVector v = VelocityVector::from_polar_components(84.6142, -2.67624);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(84.6142, EPSILON));
	}

	SECTION("Random Test 40") {
		const VelocityVector v = VelocityVector::from_polar_components(31.8507, -2.76502);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(31.8507, EPSILON));
	}

	SECTION("Random Test 41") {
		const VelocityVector v = VelocityVector::from_polar_components(38.9965, 0.789723);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(38.9965, EPSILON));
	}

	SECTION("Random Test 42") {
		const VelocityVector v = VelocityVector::from_polar_components(53.8265, 0.862028);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(53.8265, EPSILON));
	}

	SECTION("Random Test 43") {
		const VelocityVector v = VelocityVector::from_polar_components(4.98188, 0.403574);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(4.98188, EPSILON));
	}

	SECTION("Random Test 44") {
		const VelocityVector v = VelocityVector::from_polar_components(37.9088, -2.77291);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(37.9088, EPSILON));
	}

	SECTION("Random Test 45") {
		const VelocityVector v = VelocityVector::from_polar_components(31.8702, -3.13266);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(31.8702, EPSILON));
	}

	SECTION("Random Test 46") {
		const VelocityVector v = VelocityVector::from_polar_components(23.2621, 2.91791);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(23.2621, EPSILON));
	}

	SECTION("Random Test 47") {
		const VelocityVector v = VelocityVector::from_polar_components(55.4824, -1.9262);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(55.4824, EPSILON));
	}

	SECTION("Random Test 48") {
		const VelocityVector v = VelocityVector::from_polar_components(42.8617, 0.811044);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(42.8617, EPSILON));
	}

	SECTION("Random Test 49") {
		const VelocityVector v = VelocityVector::from_polar_components(95.5391, 2.56111);
		REQUIRE_THAT(v.get_magnitude(), WithinRel(95.5391, EPSILON));
	}

}

TEST_CASE("VelocityVector Utilities: get_heading", "[VelocityVector]") {
	SECTION("Random Test 0") {
		const VelocityVector v = VelocityVector::from_polar_components(72.016, 1.0091);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.0091, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector v = VelocityVector::from_polar_components(10.4815, -2.18541);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.09777, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector v = VelocityVector::from_polar_components(87.8813, -2.42852);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.85466, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector v = VelocityVector::from_polar_components(0.740005, -1.38859);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.89459, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector v = VelocityVector::from_polar_components(84.4991, 1.55203);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.55203, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector v = VelocityVector::from_polar_components(97.0152, -2.39596);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.88723, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector v = VelocityVector::from_polar_components(40.4293, -1.42477);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.85842, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector v = VelocityVector::from_polar_components(29.8333, -2.49377);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.78941, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector v = VelocityVector::from_polar_components(40.9839, -0.307957);
		REQUIRE_THAT(v.get_heading(), WithinRel(5.97523, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector v = VelocityVector::from_polar_components(83.4759, 2.33127);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.33127, EPSILON));
	}

	SECTION("Random Test 10") {
		const VelocityVector v = VelocityVector::from_polar_components(14.7477, 1.24431);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.24431, EPSILON));
	}

	SECTION("Random Test 11") {
		const VelocityVector v = VelocityVector::from_polar_components(65.484, 0.909783);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.909783, EPSILON));
	}

	SECTION("Random Test 12") {
		const VelocityVector v = VelocityVector::from_polar_components(91.4918, 1.30616);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.30616, EPSILON));
	}

	SECTION("Random Test 13") {
		const VelocityVector v = VelocityVector::from_polar_components(57.0862, -1.17384);
		REQUIRE_THAT(v.get_heading(), WithinRel(5.10934, EPSILON));
	}

	SECTION("Random Test 14") {
		const VelocityVector v = VelocityVector::from_polar_components(38.9065, -2.39888);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.88431, EPSILON));
	}

	SECTION("Random Test 15") {
		const VelocityVector v = VelocityVector::from_polar_components(59.0629, 0.649593);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.649593, EPSILON));
	}

	SECTION("Random Test 16") {
		const VelocityVector v = VelocityVector::from_polar_components(17.5267, 1.01334);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.01334, EPSILON));
	}

	SECTION("Random Test 17") {
		const VelocityVector v = VelocityVector::from_polar_components(25.4443, -0.974627);
		REQUIRE_THAT(v.get_heading(), WithinRel(5.30856, EPSILON));
	}

	SECTION("Random Test 18") {
		const VelocityVector v = VelocityVector::from_polar_components(11.0377, -2.3298);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.95339, EPSILON));
	}

	SECTION("Random Test 19") {
		const VelocityVector v = VelocityVector::from_polar_components(99.0546, -3.06614);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.21705, EPSILON));
	}

	SECTION("Random Test 20") {
		const VelocityVector v = VelocityVector::from_polar_components(88.4297, 2.50843);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.50843, EPSILON));
	}

	SECTION("Random Test 21") {
		const VelocityVector v = VelocityVector::from_polar_components(80.8918, -0.846761);
		REQUIRE_THAT(v.get_heading(), WithinRel(5.43642, EPSILON));
	}

	SECTION("Random Test 22") {
		const VelocityVector v = VelocityVector::from_polar_components(22.0531, -3.03956);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.24362, EPSILON));
	}

	SECTION("Random Test 23") {
		const VelocityVector v = VelocityVector::from_polar_components(86.5024, 0.481136);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.481136, EPSILON));
	}

	SECTION("Random Test 24") {
		const VelocityVector v = VelocityVector::from_polar_components(4.40172, 2.9234);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.9234, EPSILON));
	}

	SECTION("Random Test 25") {
		const VelocityVector v = VelocityVector::from_polar_components(43.0849, -2.36965);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.91354, EPSILON));
	}

	SECTION("Random Test 26") {
		const VelocityVector v = VelocityVector::from_polar_components(68.7464, -2.16661);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.11657, EPSILON));
	}

	SECTION("Random Test 27") {
		const VelocityVector v = VelocityVector::from_polar_components(85.8306, 3.01041);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.01041, EPSILON));
	}

	SECTION("Random Test 28") {
		const VelocityVector v = VelocityVector::from_polar_components(2.85332, -2.49873);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.78445, EPSILON));
	}

	SECTION("Random Test 29") {
		const VelocityVector v = VelocityVector::from_polar_components(22.8292, -2.01634);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.26685, EPSILON));
	}

	SECTION("Random Test 30") {
		const VelocityVector v = VelocityVector::from_polar_components(18.6605, 0.797929);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.797929, EPSILON));
	}

	SECTION("Random Test 31") {
		const VelocityVector v = VelocityVector::from_polar_components(69.0948, 2.45907);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.45907, EPSILON));
	}

	SECTION("Random Test 32") {
		const VelocityVector v = VelocityVector::from_polar_components(81.7266, 1.38222);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.38222, EPSILON));
	}

	SECTION("Random Test 33") {
		const VelocityVector v = VelocityVector::from_polar_components(20.9115, -2.48531);
		REQUIRE_THAT(v.get_heading(), WithinRel(3.79788, EPSILON));
	}

	SECTION("Random Test 34") {
		const VelocityVector v = VelocityVector::from_polar_components(14.401, 1.53787);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.53787, EPSILON));
	}

	SECTION("Random Test 35") {
		const VelocityVector v = VelocityVector::from_polar_components(89.4407, -1.41869);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.8645, EPSILON));
	}

	SECTION("Random Test 36") {
		const VelocityVector v = VelocityVector::from_polar_components(5.3625, -2.1714);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.11178, EPSILON));
	}

	SECTION("Random Test 37") {
		const VelocityVector v = VelocityVector::from_polar_components(54.9758, 2.04005);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.04005, EPSILON));
	}

	SECTION("Random Test 38") {
		const VelocityVector v = VelocityVector::from_polar_components(32.8432, 0.615539);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.615539, EPSILON));
	}

	SECTION("Random Test 39") {
		const VelocityVector v = VelocityVector::from_polar_components(85.1, 2.56461);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.56461, EPSILON));
	}

	SECTION("Random Test 40") {
		const VelocityVector v = VelocityVector::from_polar_components(61.7212, 0.699741);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.699741, EPSILON));
	}

	SECTION("Random Test 41") {
		const VelocityVector v = VelocityVector::from_polar_components(51.9116, 0.0895667);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.0895667, EPSILON));
	}

	SECTION("Random Test 42") {
		const VelocityVector v = VelocityVector::from_polar_components(67.4121, 2.94582);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.94582, EPSILON));
	}

	SECTION("Random Test 43") {
		const VelocityVector v = VelocityVector::from_polar_components(27.1155, 2.45669);
		REQUIRE_THAT(v.get_heading(), WithinRel(2.45669, EPSILON));
	}

	SECTION("Random Test 44") {
		const VelocityVector v = VelocityVector::from_polar_components(59.8832, -1.4262);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.85698, EPSILON));
	}

	SECTION("Random Test 45") {
		const VelocityVector v = VelocityVector::from_polar_components(74.8442, 1.44976);
		REQUIRE_THAT(v.get_heading(), WithinRel(1.44976, EPSILON));
	}

	SECTION("Random Test 46") {
		const VelocityVector v = VelocityVector::from_polar_components(26.945, -1.60013);
		REQUIRE_THAT(v.get_heading(), WithinRel(4.68306, EPSILON));
	}

	SECTION("Random Test 47") {
		const VelocityVector v = VelocityVector::from_polar_components(74.2323, 0.333514);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.333514, EPSILON));
	}

	SECTION("Random Test 48") {
		const VelocityVector v = VelocityVector::from_polar_components(63.6535, -1.17038);
		REQUIRE_THAT(v.get_heading(), WithinRel(5.11281, EPSILON));
	}

	SECTION("Random Test 49") {
		const VelocityVector v = VelocityVector::from_polar_components(72.4763, 0.202681);
		REQUIRE_THAT(v.get_heading(), WithinRel(0.202681, EPSILON));
	}

}

TEST_CASE("VelocityVector Utilities: angle_between", "[VelocityVector]") {
	SECTION("Standard Tests") {
		const VelocityVector v0 = VelocityVector::from_cartesian_components(0, 0);
		const VelocityVector v1 = VelocityVector::from_cartesian_components(1, 0);
		REQUIRE(v1.angle_between(v0) == 0);
		REQUIRE(v0.angle_between(v1) == 0);
		const VelocityVector v2 = VelocityVector::from_cartesian_components(0, 1);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-std::numbers::pi / 2, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(std::numbers::pi / 2, EPSILON));
		const VelocityVector v3 = VelocityVector::from_cartesian_components(1, 1);
		REQUIRE_THAT(v1.angle_between(v3), WithinRel(-std::numbers::pi / 4, EPSILON));
		REQUIRE_THAT(v3.angle_between(v1), WithinRel(std::numbers::pi / 4, EPSILON));
		REQUIRE_THAT(v2.angle_between(v3), WithinRel(std::numbers::pi / 4, EPSILON));
		REQUIRE_THAT(v3.angle_between(v2), WithinRel(-std::numbers::pi / 4, EPSILON));
	}

	SECTION("Random Test 0") {
		const VelocityVector v1 = VelocityVector::from_polar_components(49.5453, -1.03276);
		const VelocityVector v2 = VelocityVector::from_polar_components(42.9773, -0.367825);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.664933, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.664933, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector v1 = VelocityVector::from_polar_components(33.7484, -1.25956);
		const VelocityVector v2 = VelocityVector::from_polar_components(46.5472, -0.884963);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.374596, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.374596, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector v1 = VelocityVector::from_polar_components(88.7141, 0.197246);
		const VelocityVector v2 = VelocityVector::from_polar_components(72.1025, 1.62407);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.42682, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.42682, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector v1 = VelocityVector::from_polar_components(77.1633, 2.11098);
		const VelocityVector v2 = VelocityVector::from_polar_components(24.1239, 2.16673);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.0557536, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.0557536, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector v1 = VelocityVector::from_polar_components(45.8762, -2.10109);
		const VelocityVector v2 = VelocityVector::from_polar_components(47.7165, 1.85522);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.32688, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.32688, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector v1 = VelocityVector::from_polar_components(0.147795, -2.0554);
		const VelocityVector v2 = VelocityVector::from_polar_components(12.2251, -1.39882);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.656577, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.656577, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector v1 = VelocityVector::from_polar_components(54.983, 2.24925);
		const VelocityVector v2 = VelocityVector::from_polar_components(25.5758, -0.621754);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.871, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.871, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector v1 = VelocityVector::from_polar_components(20.2087, -0.697318);
		const VelocityVector v2 = VelocityVector::from_polar_components(34.6801, -0.0403028);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.657015, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.657015, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector v1 = VelocityVector::from_polar_components(65.5813, 2.39723);
		const VelocityVector v2 = VelocityVector::from_polar_components(11.9631, 2.2475);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(0.149731, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-0.149731, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector v1 = VelocityVector::from_polar_components(29.0483, -1.9777);
		const VelocityVector v2 = VelocityVector::from_polar_components(25.0951, 1.22761);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(3.07788, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-3.07788, EPSILON));
	}

	SECTION("Random Test 10") {
		const VelocityVector v1 = VelocityVector::from_polar_components(19.9246, -0.64863);
		const VelocityVector v2 = VelocityVector::from_polar_components(42.6622, 1.78587);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.4345, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.4345, EPSILON));
	}

	SECTION("Random Test 11") {
		const VelocityVector v1 = VelocityVector::from_polar_components(42.3944, -2.88004);
		const VelocityVector v2 = VelocityVector::from_polar_components(31.9973, 0.902678);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.50047, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.50047, EPSILON));
	}

	SECTION("Random Test 12") {
		const VelocityVector v1 = VelocityVector::from_polar_components(30.241, 0.697113);
		const VelocityVector v2 = VelocityVector::from_polar_components(45.5174, 2.70214);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.00503, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.00503, EPSILON));
	}

	SECTION("Random Test 13") {
		const VelocityVector v1 = VelocityVector::from_polar_components(57.9068, -2.50436);
		const VelocityVector v2 = VelocityVector::from_polar_components(77.8861, 0.519764);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-3.02412, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(3.02412, EPSILON));
	}

	SECTION("Random Test 14") {
		const VelocityVector v1 = VelocityVector::from_polar_components(68.4405, 1.59513);
		const VelocityVector v2 = VelocityVector::from_polar_components(82.7073, 1.27465);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(0.320484, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-0.320484, EPSILON));
	}

	SECTION("Random Test 15") {
		const VelocityVector v1 = VelocityVector::from_polar_components(51.1744, -0.873468);
		const VelocityVector v2 = VelocityVector::from_polar_components(19.6026, -2.22746);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(1.35399, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-1.35399, EPSILON));
	}

	SECTION("Random Test 16") {
		const VelocityVector v1 = VelocityVector::from_polar_components(2.71788, 1.91895);
		const VelocityVector v2 = VelocityVector::from_polar_components(74.6641, -0.612852);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.5318, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.5318, EPSILON));
	}

	SECTION("Random Test 17") {
		const VelocityVector v1 = VelocityVector::from_polar_components(70.0115, 1.62059);
		const VelocityVector v2 = VelocityVector::from_polar_components(6.7882, -1.87142);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.79118, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.79118, EPSILON));
	}

	SECTION("Random Test 18") {
		const VelocityVector v1 = VelocityVector::from_polar_components(8.32735, -2.28946);
		const VelocityVector v2 = VelocityVector::from_polar_components(29.7499, -0.221635);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.06782, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.06782, EPSILON));
	}

	SECTION("Random Test 19") {
		const VelocityVector v1 = VelocityVector::from_polar_components(3.86792, -1.9954);
		const VelocityVector v2 = VelocityVector::from_polar_components(53.2557, 0.118199);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.1136, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.1136, EPSILON));
	}

	SECTION("Random Test 20") {
		const VelocityVector v1 = VelocityVector::from_polar_components(88.106, -0.59433);
		const VelocityVector v2 = VelocityVector::from_polar_components(98.3664, 1.34813);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.94246, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.94246, EPSILON));
	}

	SECTION("Random Test 21") {
		const VelocityVector v1 = VelocityVector::from_polar_components(3.50573, -0.106472);
		const VelocityVector v2 = VelocityVector::from_polar_components(96.4974, 2.98116);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-3.08764, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(3.08764, EPSILON));
	}

	SECTION("Random Test 22") {
		const VelocityVector v1 = VelocityVector::from_polar_components(1.87684, -2.00976);
		const VelocityVector v2 = VelocityVector::from_polar_components(56.2484, 0.0592731);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.06903, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.06903, EPSILON));
	}

	SECTION("Random Test 23") {
		const VelocityVector v1 = VelocityVector::from_polar_components(28.4233, -2.84306);
		const VelocityVector v2 = VelocityVector::from_polar_components(53.422, -1.91877);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.92429, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.92429, EPSILON));
	}

	SECTION("Random Test 24") {
		const VelocityVector v1 = VelocityVector::from_polar_components(76.3192, 1.68365);
		const VelocityVector v2 = VelocityVector::from_polar_components(36.7856, 2.8415);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.15786, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.15786, EPSILON));
	}

	SECTION("Random Test 25") {
		const VelocityVector v1 = VelocityVector::from_polar_components(77.1052, 1.49447);
		const VelocityVector v2 = VelocityVector::from_polar_components(36.8594, -2.07386);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.71486, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.71486, EPSILON));
	}

	SECTION("Random Test 26") {
		const VelocityVector v1 = VelocityVector::from_polar_components(45.4471, -1.99462);
		const VelocityVector v2 = VelocityVector::from_polar_components(46.3554, 0.733015);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.72763, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.72763, EPSILON));
	}

	SECTION("Random Test 27") {
		const VelocityVector v1 = VelocityVector::from_polar_components(69.0671, -0.979759);
		const VelocityVector v2 = VelocityVector::from_polar_components(29.0396, 2.33687);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.96655, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.96655, EPSILON));
	}

	SECTION("Random Test 28") {
		const VelocityVector v1 = VelocityVector::from_polar_components(83.2756, 1.46481);
		const VelocityVector v2 = VelocityVector::from_polar_components(78.7893, -1.11698);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.58179, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.58179, EPSILON));
	}

	SECTION("Random Test 29") {
		const VelocityVector v1 = VelocityVector::from_polar_components(91.0011, 1.1709);
		const VelocityVector v2 = VelocityVector::from_polar_components(61.1773, 2.47588);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.30497, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.30497, EPSILON));
	}

	SECTION("Random Test 30") {
		const VelocityVector v1 = VelocityVector::from_polar_components(87.2302, -1.72645);
		const VelocityVector v2 = VelocityVector::from_polar_components(95.6327, -2.10793);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(0.381482, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-0.381482, EPSILON));
	}

	SECTION("Random Test 31") {
		const VelocityVector v1 = VelocityVector::from_polar_components(70.9937, -1.13941);
		const VelocityVector v2 = VelocityVector::from_polar_components(35.8177, 1.52087);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.66028, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.66028, EPSILON));
	}

	SECTION("Random Test 32") {
		const VelocityVector v1 = VelocityVector::from_polar_components(64.6402, 2.31405);
		const VelocityVector v2 = VelocityVector::from_polar_components(86.8183, -2.76788);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.20126, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.20126, EPSILON));
	}

	SECTION("Random Test 33") {
		const VelocityVector v1 = VelocityVector::from_polar_components(16.2396, -2.62016);
		const VelocityVector v2 = VelocityVector::from_polar_components(70.4316, 1.01934);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.64369, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.64369, EPSILON));
	}

	SECTION("Random Test 34") {
		const VelocityVector v1 = VelocityVector::from_polar_components(21.5932, -1.86982);
		const VelocityVector v2 = VelocityVector::from_polar_components(61.5988, -0.347617);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.5222, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.5222, EPSILON));
	}

	SECTION("Random Test 35") {
		const VelocityVector v1 = VelocityVector::from_polar_components(33.9288, 1.21197);
		const VelocityVector v2 = VelocityVector::from_polar_components(81.7998, -0.97265);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.18462, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.18462, EPSILON));
	}

	SECTION("Random Test 36") {
		const VelocityVector v1 = VelocityVector::from_polar_components(83.2889, -0.274887);
		const VelocityVector v2 = VelocityVector::from_polar_components(10.8418, -1.47024);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(1.19536, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-1.19536, EPSILON));
	}

	SECTION("Random Test 37") {
		const VelocityVector v1 = VelocityVector::from_polar_components(91.3889, -1.57343);
		const VelocityVector v2 = VelocityVector::from_polar_components(37.6247, 2.35461);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.35514, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.35514, EPSILON));
	}

	SECTION("Random Test 38") {
		const VelocityVector v1 = VelocityVector::from_polar_components(11.9531, -1.95001);
		const VelocityVector v2 = VelocityVector::from_polar_components(47.2878, 2.74476);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(1.58841, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-1.58841, EPSILON));
	}

	SECTION("Random Test 39") {
		const VelocityVector v1 = VelocityVector::from_polar_components(27.556, -2.84836);
		const VelocityVector v2 = VelocityVector::from_polar_components(42.1125, 2.29074);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(1.14409, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-1.14409, EPSILON));
	}

	SECTION("Random Test 40") {
		const VelocityVector v1 = VelocityVector::from_polar_components(5.18751, -2.33701);
		const VelocityVector v2 = VelocityVector::from_polar_components(12.2757, 1.70421);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(2.24196, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-2.24196, EPSILON));
	}

	SECTION("Random Test 41") {
		const VelocityVector v1 = VelocityVector::from_polar_components(19.6325, 2.56399);
		const VelocityVector v2 = VelocityVector::from_polar_components(67.4464, 1.68922);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(0.874773, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-0.874773, EPSILON));
	}

	SECTION("Random Test 42") {
		const VelocityVector v1 = VelocityVector::from_polar_components(78.5028, 2.50911);
		const VelocityVector v2 = VelocityVector::from_polar_components(2.92128, 3.03619);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.527082, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.527082, EPSILON));
	}

	SECTION("Random Test 43") {
		const VelocityVector v1 = VelocityVector::from_polar_components(47.5538, 1.77705);
		const VelocityVector v2 = VelocityVector::from_polar_components(32.1969, 1.73367);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(0.0433781, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-0.0433781, EPSILON));
	}

	SECTION("Random Test 44") {
		const VelocityVector v1 = VelocityVector::from_polar_components(72.9135, 1.04237);
		const VelocityVector v2 = VelocityVector::from_polar_components(15.913, 2.59579);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.55342, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.55342, EPSILON));
	}

	SECTION("Random Test 45") {
		const VelocityVector v1 = VelocityVector::from_polar_components(92.839, 1.5614);
		const VelocityVector v2 = VelocityVector::from_polar_components(45.6274, -2.88983);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.83196, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.83196, EPSILON));
	}

	SECTION("Random Test 46") {
		const VelocityVector v1 = VelocityVector::from_polar_components(62.2027, 1.98561);
		const VelocityVector v2 = VelocityVector::from_polar_components(21.0375, 0.868241);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(1.11737, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(-1.11737, EPSILON));
	}

	SECTION("Random Test 47") {
		const VelocityVector v1 = VelocityVector::from_polar_components(16.7099, -1.01063);
		const VelocityVector v2 = VelocityVector::from_polar_components(49.3888, 0.799227);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-1.80985, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(1.80985, EPSILON));
	}

	SECTION("Random Test 48") {
		const VelocityVector v1 = VelocityVector::from_polar_components(18.9229, -1.67313);
		const VelocityVector v2 = VelocityVector::from_polar_components(72.4578, -1.16307);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-0.510059, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(0.510059, EPSILON));
	}

	SECTION("Random Test 49") {
		const VelocityVector v1 = VelocityVector::from_polar_components(50.4503, 1.68477);
		const VelocityVector v2 = VelocityVector::from_polar_components(30.7439, -1.91379);
		REQUIRE_THAT(v1.angle_between(v2), WithinRel(-2.68463, EPSILON));
		REQUIRE_THAT(v2.angle_between(v1), WithinRel(2.68463, EPSILON));
	}

}

TEST_CASE("Aerobody: get_wind", "[Aerobody]") {
	SECTION("Standard Test: Perpendicular") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(1, 0);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(0, 1);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(std::sqrt(2), EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(std::numbers::pi / 4, EPSILON));
	}

	SECTION("Standard Test: Parallel With Car") {
		const VelocityVector reported_wind = VelocityVector::from_polar_components(1, std::numbers::pi);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(1, 0);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinAbs(0, EPSILON));
		const VelocityVector reported_wind_2 = VelocityVector::from_polar_components(10, std::numbers::pi);
		const VelocityVector car_velocity_2 = VelocityVector::from_cartesian_components(1, 0);
		const ApparentWindVector wind_2 = Aerobody::get_wind(reported_wind_2, car_velocity_2);
		REQUIRE_THAT(wind_2.speed, WithinRel(9, EPSILON));
		REQUIRE_THAT(wind_2.yaw, WithinRel(std::numbers::pi, EPSILON) || WithinRel(-std::numbers::pi, EPSILON));
	}

	SECTION("Standard Test: Parallel Against Car") {
		const VelocityVector reported_wind = VelocityVector::from_polar_components(1, 0);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(1, 0);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(2, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinAbs(0, EPSILON));
		const VelocityVector reported_wind_2 = VelocityVector::from_polar_components(10, 0);
		const VelocityVector car_velocity_2 = VelocityVector::from_cartesian_components(1, 0);
		const ApparentWindVector wind_2 = Aerobody::get_wind(reported_wind_2, car_velocity_2);		REQUIRE_THAT(wind_2.speed, WithinRel(11, EPSILON));
		REQUIRE_THAT(wind_2.yaw, WithinAbs(0, EPSILON));
	}

	SECTION("Random Test 0") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-37.1913, 13.3905);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(46.9557, -19.9461);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(11.7609, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.189564, EPSILON));
	}

	SECTION("Random Test 1") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-71.2491, 7.57487);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-5.18538, 68.3694);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(107.749, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.712916, EPSILON));
	}

	SECTION("Random Test 2") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-20.7064, -91.4614);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-61.5637, -31.5232);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(147.965, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.507983, EPSILON));
	}

	SECTION("Random Test 3") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-3.34527, 40.6631);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-20.7517, 41.3104);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(85.4419, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.179607, EPSILON));
	}

	SECTION("Random Test 4") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-46.4788, -2.30974);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(98.3507, -69.8645);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(88.8809, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.329981, EPSILON));
	}

	SECTION("Random Test 5") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-92.5561, -70.2299);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-71.1997, 12.1755);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(173.742, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.510061, EPSILON));
	}

	SECTION("Random Test 6") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(32.4893, -36.1768);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-23.8993, 8.90664);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(28.5911, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-2.23268, EPSILON));
	}

	SECTION("Random Test 7") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-5.59294, -17.7724);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-97.4142, 60.0513);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(111.346, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.16295, EPSILON));
	}

	SECTION("Random Test 8") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(0.188175, -46.0008);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(86.3569, -38.901);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(121.237, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.352571, EPSILON));
	}

	SECTION("Random Test 9") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(35.1496, 65.9155);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(20.838, 33.9003);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(114.446, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.0399679, EPSILON));
	}

	SECTION("Random Test 10") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-82.8122, 64.8273);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(12.1054, 31.701);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(119.654, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.996979, EPSILON));
	}

	SECTION("Random Test 11") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-16.5401, -8.43289);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(79.8719, -56.1338);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(90.4421, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.182452, EPSILON));
	}

	SECTION("Random Test 12") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-5.60573, -40.1246);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(4.06419, 88.3085);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(48.2086, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.0779721, EPSILON));
	}

	SECTION("Random Test 13") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(96.0433, 23.429);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-28.2852, -70.9206);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(82.7443, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-1.33896, EPSILON));
	}

	SECTION("Random Test 14") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-99.0952, 18.2738);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(27.8942, 63.1768);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(108.184, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-1.13414, EPSILON));
	}

	SECTION("Random Test 15") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(49.9866, 94.3859);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(74.1211, -32.1927);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(138.819, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.874288, EPSILON));
	}

	SECTION("Random Test 16") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-86.798, -50.6447);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-31.2732, 49.5678);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(118.076, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-1.01707, EPSILON));
	}

	SECTION("Random Test 17") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-63.7231, -67.3666);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-23.8912, -26.2832);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(128.244, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.0143539, EPSILON));
	}

	SECTION("Random Test 18") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(60.1917, 17.9197);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-52.8218, -19.6129);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(7.56192, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-2.56023, EPSILON));
	}

	SECTION("Random Test 19") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-32.5178, 87.7217);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(35.2897, -82.5675);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(5.85227, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-2.24426, EPSILON));
	}

	SECTION("Random Test 20") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-41.7151, 51.6952);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(98.6537, -8.4034);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(71.5275, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.735052, EPSILON));
	}

	SECTION("Random Test 21") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-4.5129, -44.3848);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(55.8447, -60.4215);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(116.702, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.290617, EPSILON));
	}

	SECTION("Random Test 22") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(89.8774, -81.7538);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(62.0394, -89.4882);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(228.916, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.119474, EPSILON));
	}

	SECTION("Random Test 23") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-72.4262, 79.0829);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(78.8165, -72.3356);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(9.29314, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-1.55511, EPSILON));
	}

	SECTION("Random Test 24") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-46.4133, 3.74126);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(8.24032, -68.2363);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(74.9452, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.654604, EPSILON));
	}

	SECTION("Random Test 25") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(93.7511, -9.42304);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-76.4356, 89.5945);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(82.0201, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.919021, EPSILON));
	}

	SECTION("Random Test 26") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(93.5572, 29.3297);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(3.94285, -5.31675);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(100.414, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-1.17418, EPSILON));
	}

	SECTION("Random Test 27") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-20.745, 60.6377);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-96.9224, 79.4852);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(182.976, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.185414, EPSILON));
	}

	SECTION("Random Test 28") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(84.6196, -30.9019);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-73.1535, 54.7776);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(26.4862, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(1.37578, EPSILON));
	}

	SECTION("Random Test 29") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-58.4115, -11.3213);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-36.682, -87.0273);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(136.804, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.36967, EPSILON));
	}

	SECTION("Random Test 30") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(55.3346, -35.7771);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-87.9762, -34.6034);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(77.5815, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.761796, EPSILON));
	}

	SECTION("Random Test 31") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(57.9582, -22.954);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(11.1377, 67.2181);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(82.0582, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.836842, EPSILON));
	}

	SECTION("Random Test 32") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(66.735, -14.3778);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-59.409, 94.7707);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(80.726, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.65082, EPSILON));
	}

	SECTION("Random Test 33") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(1.09514, 44.5978);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-47.0835, -90.5182);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(64.9894, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.306496, EPSILON));
	}

	SECTION("Random Test 34") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-24.5671, -75.5834);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-25.0104, -8.25476);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(97.4, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.717967, EPSILON));
	}

	SECTION("Random Test 35") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-9.38148, -42.558);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(78.1677, 32.9885);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(69.4487, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.537578, EPSILON));
	}

	SECTION("Random Test 36") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-54.5764, 36.0412);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(68.5804, -30.2815);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(15.1421, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.806004, EPSILON));
	}

	SECTION("Random Test 37") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(39.3531, 32.7517);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-99.1828, -84.6005);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(79.1701, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.00783754, EPSILON));
	}

	SECTION("Random Test 38") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-60.3158, 33.5305);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(11.7255, -52.2431);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(52.069, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(1.42397, EPSILON));
	}

	SECTION("Random Test 39") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-22.3857, -6.06471);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-78.2996, -9.16792);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(101.831, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.0335937, EPSILON));
	}

	SECTION("Random Test 40") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(25.9297, 21.7219);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-53.1645, -58.8585);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(46.0528, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.101841, EPSILON));
	}

	SECTION("Random Test 41") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-71.1277, -58.6188);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(6.19631, -4.1892);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(90.3379, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(1.77832, EPSILON));
	}

	SECTION("Random Test 42") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-58.7103, 99.3056);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(30.7188, -92.6048);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(28.7824, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(2.12606, EPSILON));
	}

	SECTION("Random Test 43") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(70.3685, 49.8081);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-77.5049, 14.7748);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(64.976, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(1.27237, EPSILON));
	}

	SECTION("Random Test 44") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(96.6762, -18.9072);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-87.3636, 33.456);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(17.2741, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(1.77445, EPSILON));
	}

	SECTION("Random Test 45") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(60.2252, -55.2488);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(73.6704, 48.3894);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(134.071, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.632351, EPSILON));
	}

	SECTION("Random Test 46") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-14.5111, 84.6906);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(29.9065, 79.9737);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(165.382, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.264628, EPSILON));
	}

	SECTION("Random Test 47") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(62.2111, -10.1086);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(96.4909, 43.9141);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(162.263, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(0.217221, EPSILON));
	}

	SECTION("Random Test 48") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(91.9404, -83.6767);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-10.8584, -40.2499);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(148.095, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.842877, EPSILON));
	}

	SECTION("Random Test 49") {
		const VelocityVector reported_wind = VelocityVector::from_cartesian_components(-81.3329, -27.8547);
		const VelocityVector car_velocity = VelocityVector::from_cartesian_components(-70.1687, 37.8171);
		const ApparentWindVector wind = Aerobody::get_wind(reported_wind, car_velocity);
		REQUIRE_THAT(wind.speed, WithinRel(151.829, EPSILON));
		REQUIRE_THAT(wind.yaw, WithinRel(-0.428653, EPSILON));
	}

}

TEST_CASE("Aerobody: aerodynamic_drag", "[Aerobody]") {
	SECTION("Standard Test: Perpendicular") {
		const double drag_coefficient = 0.005;
		const double frontal_area = 1;
		const double air_density = 1.225;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind_perp = { .speed = 1, .yaw = 0};
		const double drag = aero.aerodynamic_drag(wind_perp, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0030625, EPSILON));
		const ApparentWindVector wind_perp_2 = { .speed = 10, .yaw = 0};
		const double drag_2 = aero.aerodynamic_drag(wind_perp_2, air_density);
		REQUIRE_THAT(drag_2, WithinRel(0.30625, EPSILON));
	}

	SECTION("Standard Test: Parallel With Car") {
		const double drag_coefficient = 0.005;
		const double frontal_area = 1;
		const double air_density = 1.225;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind_par = { .speed = 1, .yaw = std::numbers::pi / 2};
		const double drag = aero.aerodynamic_drag(wind_par, air_density);
		REQUIRE_THAT(drag, WithinAbs(0, EPSILON / 100.0));
		const ApparentWindVector wind_par_2 = { .speed = 10, .yaw = std::numbers::pi / 2};
		const double drag_2 = aero.aerodynamic_drag(wind_par_2, air_density);
		REQUIRE_THAT(drag_2, WithinAbs(0, EPSILON / 100.0));
	}

	SECTION("Random Test 0") {
		const double drag_coefficient = 0.00485082;
		const double frontal_area = 9.96527;
		const double air_density = 1.0114;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 87.1419, .yaw = 2.90483};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(175.419, EPSILON));
	}

	SECTION("Random Test 1") {
		const double drag_coefficient = 0.00142748;
		const double frontal_area = 1.03851;
		const double air_density = 1.47836;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 71.1013, .yaw = -2.62568};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(4.19149, EPSILON));
	}

	SECTION("Random Test 2") {
		const double drag_coefficient = 0.00604104;
		const double frontal_area = 6.4018;
		const double air_density = 1.3037;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 82.7738, .yaw = -1.78674};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(7.93024, EPSILON));
	}

	SECTION("Random Test 3") {
		const double drag_coefficient = 0.000502963;
		const double frontal_area = 1.56851;
		const double air_density = 1.16699;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 15.792, .yaw = 1.84016};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.00812967, EPSILON));
	}

	SECTION("Random Test 4") {
		const double drag_coefficient = 0.00905745;
		const double frontal_area = 6.83938;
		const double air_density = 1.1603;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 12.5483, .yaw = -0.170801};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(5.4954, EPSILON));
	}

	SECTION("Random Test 5") {
		const double drag_coefficient = 0.0059031;
		const double frontal_area = 7.56567;
		const double air_density = 1.01334;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 40.834, .yaw = -1.07204};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(8.63308, EPSILON));
	}

	SECTION("Random Test 6") {
		const double drag_coefficient = 0.00846982;
		const double frontal_area = 6.65448;
		const double air_density = 1.02078;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 69.1143, .yaw = 0.894515};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(53.8309, EPSILON));
	}

	SECTION("Random Test 7") {
		const double drag_coefficient = 0.0043904;
		const double frontal_area = 9.79455;
		const double air_density = 1.33352;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 5.67796, .yaw = -3.03726};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.914334, EPSILON));
	}

	SECTION("Random Test 8") {
		const double drag_coefficient = 0.00589552;
		const double frontal_area = 3.09407;
		const double air_density = 1.23487;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 64.0923, .yaw = -3.034};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(45.7318, EPSILON));
	}

	SECTION("Random Test 9") {
		const double drag_coefficient = 0.00852294;
		const double frontal_area = 2.291;
		const double air_density = 1.08042;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 70.7549, .yaw = -1.73619};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.43147, EPSILON));
	}

	SECTION("Random Test 10") {
		const double drag_coefficient = 0.00314387;
		const double frontal_area = 3.80293;
		const double air_density = 1.01981;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 16.8815, .yaw = -2.43994};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.0135, EPSILON));
	}

	SECTION("Random Test 11") {
		const double drag_coefficient = 0.00687091;
		const double frontal_area = 2.90028;
		const double air_density = 1.28127;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 7.15981, .yaw = 2.74708};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.557758, EPSILON));
	}

	SECTION("Random Test 12") {
		const double drag_coefficient = 0.00463421;
		const double frontal_area = 7.18087;
		const double air_density = 1.23101;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 75.0044, .yaw = -0.703943};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(66.9584, EPSILON));
	}

	SECTION("Random Test 13") {
		const double drag_coefficient = 0.00753585;
		const double frontal_area = 2.39591;
		const double air_density = 1.37569;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 41.2235, .yaw = 2.90523};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(19.9476, EPSILON));
	}

	SECTION("Random Test 14") {
		const double drag_coefficient = 0.00239729;
		const double frontal_area = 5.06191;
		const double air_density = 1.07529;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 41.1108, .yaw = 0.148852};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(10.7841, EPSILON));
	}

	SECTION("Random Test 15") {
		const double drag_coefficient = 0.00364894;
		const double frontal_area = 9.576;
		const double air_density = 1.38224;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 31.4893, .yaw = -0.0863278};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(23.7678, EPSILON));
	}

	SECTION("Random Test 16") {
		const double drag_coefficient = 0.00757374;
		const double frontal_area = 5.4653;
		const double air_density = 1.45207;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 1.34356, .yaw = -2.14184};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0158494, EPSILON));
	}

	SECTION("Random Test 17") {
		const double drag_coefficient = 0.00934383;
		const double frontal_area = 4.38962;
		const double air_density = 1.07498;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 61.8852, .yaw = 1.13139};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(15.2788, EPSILON));
	}

	SECTION("Random Test 18") {
		const double drag_coefficient = 0.00078809;
		const double frontal_area = 7.01144;
		const double air_density = 1.0493;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 43.5275, .yaw = 2.05574};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.19358, EPSILON));
	}

	SECTION("Random Test 19") {
		const double drag_coefficient = 0.000637604;
		const double frontal_area = 4.75322;
		const double air_density = 1.46536;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 73.372, .yaw = -0.100479};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(11.8337, EPSILON));
	}

	SECTION("Random Test 20") {
		const double drag_coefficient = 0.00548633;
		const double frontal_area = 1.30792;
		const double air_density = 1.06959;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 98.8745, .yaw = -2.51609};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(24.6552, EPSILON));
	}

	SECTION("Random Test 21") {
		const double drag_coefficient = 0.00654405;
		const double frontal_area = 4.03322;
		const double air_density = 1.15396;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 30.964, .yaw = -2.64173};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(11.2464, EPSILON));
	}

	SECTION("Random Test 22") {
		const double drag_coefficient = 0.00447924;
		const double frontal_area = 4.37927;
		const double air_density = 1.00834;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 75.1769, .yaw = 2.76476};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(48.3237, EPSILON));
	}

	SECTION("Random Test 23") {
		const double drag_coefficient = 0.00599344;
		const double frontal_area = 6.09071;
		const double air_density = 1.19286;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 58.592, .yaw = -0.346867};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(66.1065, EPSILON));
	}

	SECTION("Random Test 24") {
		const double drag_coefficient = 0.0036236;
		const double frontal_area = 2.43757;
		const double air_density = 1.40913;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 13.9771, .yaw = 2.57202};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.862219, EPSILON));
	}

	SECTION("Random Test 25") {
		const double drag_coefficient = 0.00580884;
		const double frontal_area = 8.44446;
		const double air_density = 1.37147;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 48.6884, .yaw = 1.25974};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(7.46975, EPSILON));
	}

	SECTION("Random Test 26") {
		const double drag_coefficient = 0.00524536;
		const double frontal_area = 4.54778;
		const double air_density = 1.45011;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 18.0258, .yaw = 2.34696};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.75812, EPSILON));
	}

	SECTION("Random Test 27") {
		const double drag_coefficient = 0.00337276;
		const double frontal_area = 5.46329;
		const double air_density = 1.30065;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 26.4036, .yaw = -1.92166};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.986933, EPSILON));
	}

	SECTION("Random Test 28") {
		const double drag_coefficient = 0.00959308;
		const double frontal_area = 7.31831;
		const double air_density = 1.49445;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 7.24173, .yaw = 0.858811};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.1743, EPSILON));
	}

	SECTION("Random Test 29") {
		const double drag_coefficient = 0.00795614;
		const double frontal_area = 7.23671;
		const double air_density = 1.37392;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 33.1252, .yaw = -2.98013};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(42.2786, EPSILON));
	}

	SECTION("Random Test 30") {
		const double drag_coefficient = 0.00791853;
		const double frontal_area = 7.62116;
		const double air_density = 1.13085;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 77.1782, .yaw = 2.66874};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(161.092, EPSILON));
	}

	SECTION("Random Test 31") {
		const double drag_coefficient = 0.00261655;
		const double frontal_area = 9.44851;
		const double air_density = 1.19446;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 32.1084, .yaw = 0.256624};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(14.2414, EPSILON));
	}

	SECTION("Random Test 32") {
		const double drag_coefficient = 0.00756787;
		const double frontal_area = 1.97857;
		const double air_density = 1.48641;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 63.6996, .yaw = 1.8682};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(3.87768, EPSILON));
	}

	SECTION("Random Test 33") {
		const double drag_coefficient = 0.000552675;
		const double frontal_area = 8.98982;
		const double air_density = 1.38162;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 2.67146, .yaw = 1.46302};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.000283415, EPSILON));
	}

	SECTION("Random Test 34") {
		const double drag_coefficient = 0.00749894;
		const double frontal_area = 6.11961;
		const double air_density = 1.012;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 48.9948, .yaw = 2.65681};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(43.6355, EPSILON));
	}

	SECTION("Random Test 35") {
		const double drag_coefficient = 0.000720561;
		const double frontal_area = 7.28605;
		const double air_density = 1.36808;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 0.215518, .yaw = -2.53166};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.000112074, EPSILON));
	}

	SECTION("Random Test 36") {
		const double drag_coefficient = 0.00324537;
		const double frontal_area = 5.09214;
		const double air_density = 1.02262;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 35.6238, .yaw = -1.83583};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.735767, EPSILON));
	}

	SECTION("Random Test 37") {
		const double drag_coefficient = 0.00719305;
		const double frontal_area = 6.59958;
		const double air_density = 1.15667;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 3.50103, .yaw = -1.31218};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0220092, EPSILON));
	}

	SECTION("Random Test 38") {
		const double drag_coefficient = 0.00790473;
		const double frontal_area = 8.39365;
		const double air_density = 1.00131;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 44.0864, .yaw = 3.03149};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(63.7838, EPSILON));
	}

	SECTION("Random Test 39") {
		const double drag_coefficient = 0.00728517;
		const double frontal_area = 2.36921;
		const double air_density = 1.04891;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 55.2111, .yaw = 1.73128};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.704548, EPSILON));
	}

	SECTION("Random Test 40") {
		const double drag_coefficient = 0.00803959;
		const double frontal_area = 6.86602;
		const double air_density = 1.17554;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 16.2008, .yaw = 2.47705};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(5.27702, EPSILON));
	}

	SECTION("Random Test 41") {
		const double drag_coefficient = 0.00949794;
		const double frontal_area = 4.49495;
		const double air_density = 1.21492;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 46.2403, .yaw = 0.52048};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(41.738, EPSILON));
	}

	SECTION("Random Test 42") {
		const double drag_coefficient = 0.00203765;
		const double frontal_area = 5.03728;
		const double air_density = 1.3515;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 87.8795, .yaw = 0.619651};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(35.4994, EPSILON));
	}

	SECTION("Random Test 43") {
		const double drag_coefficient = 0.00298688;
		const double frontal_area = 1.93343;
		const double air_density = 1.31739;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 74.8794, .yaw = -0.438502};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(17.4834, EPSILON));
	}

	SECTION("Random Test 44") {
		const double drag_coefficient = 0.00367008;
		const double frontal_area = 5.02199;
		const double air_density = 1.30922;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 5.43578, .yaw = -2.1897};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.119984, EPSILON));
	}

	SECTION("Random Test 45") {
		const double drag_coefficient = 0.00706019;
		const double frontal_area = 5.46983;
		const double air_density = 1.324;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 30.6727, .yaw = -1.19175};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(3.29326, EPSILON));
	}

	SECTION("Random Test 46") {
		const double drag_coefficient = 0.00302315;
		const double frontal_area = 6.16909;
		const double air_density = 1.08966;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 75.1153, .yaw = -0.924032};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(20.8194, EPSILON));
	}

	SECTION("Random Test 47") {
		const double drag_coefficient = 0.00746043;
		const double frontal_area = 4.52509;
		const double air_density = 1.19736;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 72.1319, .yaw = 1.98722};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(17.2056, EPSILON));
	}

	SECTION("Random Test 48") {
		const double drag_coefficient = 0.00693471;
		const double frontal_area = 2.13397;
		const double air_density = 1.04292;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 6.3094, .yaw = -3.05491};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.304891, EPSILON));
	}

	SECTION("Random Test 49") {
		const double drag_coefficient = 0.00838978;
		const double frontal_area = 3.75798;
		const double air_density = 1.44933;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 13.2622, .yaw = -2.64234};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(3.09743, EPSILON));
	}

	SECTION("Random Test 50") {
		const double drag_coefficient = 0.00468283;
		const double frontal_area = 8.58176;
		const double air_density = 1.43047;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 9.20646, .yaw = -0.0315809};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.4338, EPSILON));
	}

	SECTION("Random Test 51") {
		const double drag_coefficient = 0.00891947;
		const double frontal_area = 7.83842;
		const double air_density = 1.08525;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 45.1059, .yaw = -1.38129};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.73886, EPSILON));
	}

	SECTION("Random Test 52") {
		const double drag_coefficient = 0.000195692;
		const double frontal_area = 5.70475;
		const double air_density = 1.07413;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 24.6848, .yaw = 0.417974};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.305145, EPSILON));
	}

	SECTION("Random Test 53") {
		const double drag_coefficient = 0.00441445;
		const double frontal_area = 4.80876;
		const double air_density = 1.01129;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 28.455, .yaw = -1.69651};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.136641, EPSILON));
	}

	SECTION("Random Test 54") {
		const double drag_coefficient = 0.00071644;
		const double frontal_area = 4.77827;
		const double air_density = 1.33358;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 68.3589, .yaw = -1.65409};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0738369, EPSILON));
	}

	SECTION("Random Test 55") {
		const double drag_coefficient = 0.00013081;
		const double frontal_area = 5.40942;
		const double air_density = 1.00177;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 50.6865, .yaw = -2.1515};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.274061, EPSILON));
	}

	SECTION("Random Test 56") {
		const double drag_coefficient = 0.00089402;
		const double frontal_area = 4.81769;
		const double air_density = 1.20066;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 60.8245, .yaw = -2.1307};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.6983, EPSILON));
	}

	SECTION("Random Test 57") {
		const double drag_coefficient = 0.00294532;
		const double frontal_area = 8.41401;
		const double air_density = 1.03602;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 20.0566, .yaw = 2.82771};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(4.67177, EPSILON));
	}

	SECTION("Random Test 58") {
		const double drag_coefficient = 0.00537564;
		const double frontal_area = 2.00089;
		const double air_density = 1.07057;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 70.6886, .yaw = -1.38601};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.971198, EPSILON));
	}

	SECTION("Random Test 59") {
		const double drag_coefficient = 0.00446776;
		const double frontal_area = 2.92255;
		const double air_density = 1.21146;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 1.44261, .yaw = 2.83882};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0149967, EPSILON));
	}

	SECTION("Random Test 60") {
		const double drag_coefficient = 0.00330343;
		const double frontal_area = 9.15064;
		const double air_density = 1.14483;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 53.8953, .yaw = 2.9624};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(48.6642, EPSILON));
	}

	SECTION("Random Test 61") {
		const double drag_coefficient = 0.00956625;
		const double frontal_area = 4.21451;
		const double air_density = 1.09988;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 7.83629, .yaw = 1.7068};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0250282, EPSILON));
	}

	SECTION("Random Test 62") {
		const double drag_coefficient = 0.00498678;
		const double frontal_area = 7.1475;
		const double air_density = 1.10142;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 35.3997, .yaw = -2.3967};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(13.2942, EPSILON));
	}

	SECTION("Random Test 63") {
		const double drag_coefficient = 0.0084599;
		const double frontal_area = 6.37882;
		const double air_density = 1.01917;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 18.0189, .yaw = 2.02243};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.70063, EPSILON));
	}

	SECTION("Random Test 64") {
		const double drag_coefficient = 0.00444533;
		const double frontal_area = 9.56265;
		const double air_density = 1.41995;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 71.5106, .yaw = 2.36856};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(79.0765, EPSILON));
	}

	SECTION("Random Test 65") {
		const double drag_coefficient = 0.00474491;
		const double frontal_area = 7.40289;
		const double air_density = 1.09325;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 78.452, .yaw = -0.68487};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(70.8875, EPSILON));
	}

	SECTION("Random Test 66") {
		const double drag_coefficient = 0.00177631;
		const double frontal_area = 6.74237;
		const double air_density = 1.16847;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 45.2729, .yaw = 1.1072};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.8677, EPSILON));
	}

	SECTION("Random Test 67") {
		const double drag_coefficient = 0.00235386;
		const double frontal_area = 9.97601;
		const double air_density = 1.31362;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 40.1821, .yaw = -1.81931};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.50654, EPSILON));
	}

	SECTION("Random Test 68") {
		const double drag_coefficient = 0.00422101;
		const double frontal_area = 2.51883;
		const double air_density = 1.33516;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 64.9082, .yaw = -2.30069};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(13.2954, EPSILON));
	}

	SECTION("Random Test 69") {
		const double drag_coefficient = 0.00435754;
		const double frontal_area = 3.84016;
		const double air_density = 1.16567;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 19.9172, .yaw = -1.8384};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.270513, EPSILON));
	}

	SECTION("Random Test 70") {
		const double drag_coefficient = 0.00775207;
		const double frontal_area = 5.07593;
		const double air_density = 1.49957;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 94.21, .yaw = -2.99668};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(256.395, EPSILON));
	}

	SECTION("Random Test 71") {
		const double drag_coefficient = 0.00836634;
		const double frontal_area = 8.48738;
		const double air_density = 1.41501;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 53.732, .yaw = 1.59619};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0934933, EPSILON));
	}

	SECTION("Random Test 72") {
		const double drag_coefficient = 0.00106022;
		const double frontal_area = 4.76056;
		const double air_density = 1.40641;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 29.7247, .yaw = -2.21123};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(1.11972, EPSILON));
	}

	SECTION("Random Test 73") {
		const double drag_coefficient = 0.00523749;
		const double frontal_area = 8.95823;
		const double air_density = 1.31439;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 18.641, .yaw = 0.731189};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(5.93702, EPSILON));
	}

	SECTION("Random Test 74") {
		const double drag_coefficient = 0.000418969;
		const double frontal_area = 5.09264;
		const double air_density = 1.30688;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 42.8406, .yaw = -0.376878};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(2.21227, EPSILON));
	}

	SECTION("Random Test 75") {
		const double drag_coefficient = 0.000600733;
		const double frontal_area = 7.94855;
		const double air_density = 1.01466;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 45.2538, .yaw = 0.302168};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(4.52168, EPSILON));
	}

	SECTION("Random Test 76") {
		const double drag_coefficient = 0.0094675;
		const double frontal_area = 7.48695;
		const double air_density = 1.04911;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 91.9138, .yaw = -0.583074};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(218.893, EPSILON));
	}

	SECTION("Random Test 77") {
		const double drag_coefficient = 0.00675344;
		const double frontal_area = 7.28419;
		const double air_density = 1.13923;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 55.3804, .yaw = -0.378951};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(74.1793, EPSILON));
	}

	SECTION("Random Test 78") {
		const double drag_coefficient = 0.000103379;
		const double frontal_area = 7.73956;
		const double air_density = 1.06971;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 5.06879, .yaw = -2.27787};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.00463981, EPSILON));
	}

	SECTION("Random Test 79") {
		const double drag_coefficient = 0.000802695;
		const double frontal_area = 4.39434;
		const double air_density = 1.17695;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 8.88585, .yaw = -2.55982};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.114406, EPSILON));
	}

	SECTION("Random Test 80") {
		const double drag_coefficient = 0.00203516;
		const double frontal_area = 3.86429;
		const double air_density = 1.23619;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 4.55783, .yaw = -2.70302};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0827718, EPSILON));
	}

	SECTION("Random Test 81") {
		const double drag_coefficient = 0.00906523;
		const double frontal_area = 2.5464;
		const double air_density = 1.41113;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 4.85865, .yaw = 2.2466};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.15044, EPSILON));
	}

	SECTION("Random Test 82") {
		const double drag_coefficient = 0.00946174;
		const double frontal_area = 4.48251;
		const double air_density = 1.17789;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 88.8308, .yaw = 1.77004};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(7.72136, EPSILON));
	}

	SECTION("Random Test 83") {
		const double drag_coefficient = 0.00918383;
		const double frontal_area = 7.08157;
		const double air_density = 1.00704;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 38.2015, .yaw = -1.17046};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(7.25863, EPSILON));
	}

	SECTION("Random Test 84") {
		const double drag_coefficient = 0.00907353;
		const double frontal_area = 8.08907;
		const double air_density = 1.44263;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 89.1471, .yaw = 1.9574};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(59.8126, EPSILON));
	}

	SECTION("Random Test 85") {
		const double drag_coefficient = 0.00816603;
		const double frontal_area = 3.74403;
		const double air_density = 1.47273;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 80.4432, .yaw = 2.75926};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(125.409, EPSILON));
	}

	SECTION("Random Test 86") {
		const double drag_coefficient = 0.00479377;
		const double frontal_area = 1.39994;
		const double air_density = 1.43373;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 57.0674, .yaw = -1.71114};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.306562, EPSILON));
	}

	SECTION("Random Test 87") {
		const double drag_coefficient = 0.00602757;
		const double frontal_area = 4.36407;
		const double air_density = 1.09623;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 98.3807, .yaw = 1.88755};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(13.5391, EPSILON));
	}

	SECTION("Random Test 88") {
		const double drag_coefficient = 0.00895279;
		const double frontal_area = 3.03147;
		const double air_density = 1.08169;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 81.4258, .yaw = -1.0806};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(21.5712, EPSILON));
	}

	SECTION("Random Test 89") {
		const double drag_coefficient = 0.0086814;
		const double frontal_area = 3.75636;
		const double air_density = 1.45412;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 11.514, .yaw = -1.66191};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0260248, EPSILON));
	}

	SECTION("Random Test 90") {
		const double drag_coefficient = 0.000715513;
		const double frontal_area = 6.61224;
		const double air_density = 1.16703;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 7.80353, .yaw = 1.33779};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.00896334, EPSILON));
	}

	SECTION("Random Test 91") {
		const double drag_coefficient = 0.0042833;
		const double frontal_area = 6.01738;
		const double air_density = 1.12022;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 84.2877, .yaw = -0.263492};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(95.6049, EPSILON));
	}

	SECTION("Random Test 92") {
		const double drag_coefficient = 0.00635689;
		const double frontal_area = 1.36761;
		const double air_density = 1.20935;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 32.8845, .yaw = 2.88495};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(5.31846, EPSILON));
	}

	SECTION("Random Test 93") {
		const double drag_coefficient = 0.00668535;
		const double frontal_area = 5.79797;
		const double air_density = 1.18971;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 27.4218, .yaw = -1.39559};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.526826, EPSILON));
	}

	SECTION("Random Test 94") {
		const double drag_coefficient = 0.00603349;
		const double frontal_area = 1.34858;
		const double air_density = 1.48981;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 98.7653, .yaw = -1.68001};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.702334, EPSILON));
	}

	SECTION("Random Test 95") {
		const double drag_coefficient = 0.00347139;
		const double frontal_area = 8.17723;
		const double air_density = 1.3386;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 35.9546, .yaw = 1.00444};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(7.071, EPSILON));
	}

	SECTION("Random Test 96") {
		const double drag_coefficient = 0.0074159;
		const double frontal_area = 6.90134;
		const double air_density = 1.003;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 34.2048, .yaw = 1.61471};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.0578622, EPSILON));
	}

	SECTION("Random Test 97") {
		const double drag_coefficient = 0.00872982;
		const double frontal_area = 8.93477;
		const double air_density = 1.46806;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 14.0201, .yaw = 0.3844};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(9.6713, EPSILON));
	}

	SECTION("Random Test 98") {
		const double drag_coefficient = 0.00906977;
		const double frontal_area = 7.84539;
		const double air_density = 1.09951;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 9.62145, .yaw = -1.1248};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(0.673795, EPSILON));
	}

	SECTION("Random Test 99") {
		const double drag_coefficient = 0.000986118;
		const double frontal_area = 8.00025;
		const double air_density = 1.03797;
		const Aerobody aero = Aerobody(drag_coefficient, frontal_area);
		const ApparentWindVector wind = { .speed = 43.4583, .yaw = 0.758181};
		const double drag = aero.aerodynamic_drag(wind, air_density);
		REQUIRE_THAT(drag, WithinRel(4.07674, EPSILON));
	}

}

