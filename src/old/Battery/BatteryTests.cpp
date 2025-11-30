#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "Battery.h"
#include "BatteryState.h"

constexpr double EPSILON = 0.001; // %

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("BatteryState: update_energy_remaining", "[BatteryState]") {
	SECTION("Standard Tests") {
		auto a = BatteryState(100);
		a.update_energy_remaining(100);
		REQUIRE_THAT(a.get_energy_remaining(), WithinAbs(200.0, EPSILON));
		a.update_energy_remaining(-100);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(100.0, EPSILON));
		a.update_energy_remaining(1000);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1100.0, EPSILON));
		a.update_energy_remaining(0);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1100.0, EPSILON));
	}

	SECTION("Random Test 0") {
		auto a = BatteryState(1352.33);
		a.update_energy_remaining(-155.581);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1196.75, EPSILON));
	}
	SECTION("Random Test 1") {
		auto a = BatteryState(1015.07);
		a.update_energy_remaining(-793.135);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(221.932, EPSILON));
	}
	SECTION("Random Test 2") {
		auto a = BatteryState(2351.35);
		a.update_energy_remaining(592.909);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2944.26, EPSILON));
	}
	SECTION("Random Test 3") {
		auto a = BatteryState(662.528);
		a.update_energy_remaining(862.038);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1524.57, EPSILON));
	}
	SECTION("Random Test 4") {
		auto a = BatteryState(2340.4);
		a.update_energy_remaining(750.54);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3090.94, EPSILON));
	}
	SECTION("Random Test 5") {
		auto a = BatteryState(3678.76);
		a.update_energy_remaining(-116.35);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3562.41, EPSILON));
	}
	SECTION("Random Test 6") {
		auto a = BatteryState(2242.41);
		a.update_energy_remaining(537.668);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2780.08, EPSILON));
	}
	SECTION("Random Test 7") {
		auto a = BatteryState(5242.73);
		a.update_energy_remaining(-935.377);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4307.36, EPSILON));
	}
	SECTION("Random Test 8") {
		auto a = BatteryState(2441.74);
		a.update_energy_remaining(-808.22);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1633.52, EPSILON));
	}
	SECTION("Random Test 9") {
		auto a = BatteryState(544.185);
		a.update_energy_remaining(946.479);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1490.66, EPSILON));
	}
	SECTION("Random Test 10") {
		auto a = BatteryState(4701.82);
		a.update_energy_remaining(559.023);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5260.84, EPSILON));
	}
	SECTION("Random Test 11") {
		auto a = BatteryState(5326.65);
		a.update_energy_remaining(105.076);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5431.72, EPSILON));
	}
	SECTION("Random Test 12") {
		auto a = BatteryState(7810.14);
		a.update_energy_remaining(302.059);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(8112.2, EPSILON));
	}
	SECTION("Random Test 13") {
		auto a = BatteryState(5729.59);
		a.update_energy_remaining(-454.709);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5274.88, EPSILON));
	}
	SECTION("Random Test 14") {
		auto a = BatteryState(1602.97);
		a.update_energy_remaining(48.3929);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1651.36, EPSILON));
	}
	SECTION("Random Test 15") {
		auto a = BatteryState(1559.23);
		a.update_energy_remaining(-379.456);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1179.78, EPSILON));
	}
	SECTION("Random Test 16") {
		auto a = BatteryState(5365.65);
		a.update_energy_remaining(-91.8256);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5273.82, EPSILON));
	}
	SECTION("Random Test 17") {
		auto a = BatteryState(6980.09);
		a.update_energy_remaining(-476.207);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6503.88, EPSILON));
	}
	SECTION("Random Test 18") {
		auto a = BatteryState(412.333);
		a.update_energy_remaining(-324.705);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(87.6282, EPSILON));
	}
	SECTION("Random Test 19") {
		auto a = BatteryState(5145.06);
		a.update_energy_remaining(-191.084);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4953.98, EPSILON));
	}
	SECTION("Random Test 20") {
		auto a = BatteryState(3205.01);
		a.update_energy_remaining(919.041);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4124.05, EPSILON));
	}
	SECTION("Random Test 21") {
		auto a = BatteryState(2352.09);
		a.update_energy_remaining(826.775);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3178.87, EPSILON));
	}
	SECTION("Random Test 22") {
		auto a = BatteryState(1340.77);
		a.update_energy_remaining(579.634);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1920.4, EPSILON));
	}
	SECTION("Random Test 23") {
		auto a = BatteryState(6235.41);
		a.update_energy_remaining(793.562);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7028.97, EPSILON));
	}
	SECTION("Random Test 24") {
		auto a = BatteryState(5802.95);
		a.update_energy_remaining(-4.74931);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5798.2, EPSILON));
	}
	SECTION("Random Test 25") {
		auto a = BatteryState(1336.3);
		a.update_energy_remaining(670.725);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2007.02, EPSILON));
	}
	SECTION("Random Test 26") {
		auto a = BatteryState(3091.65);
		a.update_energy_remaining(920.478);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4012.12, EPSILON));
	}
	SECTION("Random Test 27") {
		auto a = BatteryState(5400.96);
		a.update_energy_remaining(-301.233);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5099.73, EPSILON));
	}
	SECTION("Random Test 28") {
		auto a = BatteryState(854.934);
		a.update_energy_remaining(-812.494);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(42.4396, EPSILON));
	}
	SECTION("Random Test 29") {
		auto a = BatteryState(6747.21);
		a.update_energy_remaining(-558.401);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6188.81, EPSILON));
	}
	SECTION("Random Test 30") {
		auto a = BatteryState(7163.25);
		a.update_energy_remaining(-126.248);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7037, EPSILON));
	}
	SECTION("Random Test 31") {
		auto a = BatteryState(2457.15);
		a.update_energy_remaining(-799.258);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1657.89, EPSILON));
	}
	SECTION("Random Test 32") {
		auto a = BatteryState(3770.02);
		a.update_energy_remaining(587.086);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4357.1, EPSILON));
	}
	SECTION("Random Test 33") {
		auto a = BatteryState(6078.54);
		a.update_energy_remaining(-969.8);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5108.74, EPSILON));
	}
	SECTION("Random Test 34") {
		auto a = BatteryState(7798.18);
		a.update_energy_remaining(55.0154);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7853.2, EPSILON));
	}
	SECTION("Random Test 35") {
		auto a = BatteryState(1093.26);
		a.update_energy_remaining(3.86067);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1097.12, EPSILON));
	}
	SECTION("Random Test 36") {
		auto a = BatteryState(1897.32);
		a.update_energy_remaining(648.015);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2545.33, EPSILON));
	}
	SECTION("Random Test 37") {
		auto a = BatteryState(1510.31);
		a.update_energy_remaining(961.73);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2472.04, EPSILON));
	}
	SECTION("Random Test 38") {
		auto a = BatteryState(7129.84);
		a.update_energy_remaining(-405.463);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6724.37, EPSILON));
	}
	SECTION("Random Test 39") {
		auto a = BatteryState(1230.8);
		a.update_energy_remaining(347.004);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1577.81, EPSILON));
	}
	SECTION("Random Test 40") {
		auto a = BatteryState(1131.21);
		a.update_energy_remaining(337.82);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1469.03, EPSILON));
	}
	SECTION("Random Test 41") {
		auto a = BatteryState(8001.11);
		a.update_energy_remaining(687.164);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(8688.28, EPSILON));
	}
	SECTION("Random Test 42") {
		auto a = BatteryState(6420.36);
		a.update_energy_remaining(-198.801);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6221.56, EPSILON));
	}
	SECTION("Random Test 43") {
		auto a = BatteryState(6162.25);
		a.update_energy_remaining(620.7);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6782.95, EPSILON));
	}
	SECTION("Random Test 44") {
		auto a = BatteryState(5865.38);
		a.update_energy_remaining(377.582);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6242.96, EPSILON));
	}
	SECTION("Random Test 45") {
		auto a = BatteryState(5534.35);
		a.update_energy_remaining(-981.042);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4553.31, EPSILON));
	}
	SECTION("Random Test 46") {
		auto a = BatteryState(2922.27);
		a.update_energy_remaining(914.098);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3836.37, EPSILON));
	}
	SECTION("Random Test 47") {
		auto a = BatteryState(6018.04);
		a.update_energy_remaining(677.018);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6695.06, EPSILON));
	}
	SECTION("Random Test 48") {
		auto a = BatteryState(1240.86);
		a.update_energy_remaining(-775.435);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(465.427, EPSILON));
	}
	SECTION("Random Test 49") {
		auto a = BatteryState(5125.7);
		a.update_energy_remaining(-924.059);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4201.64, EPSILON));
	}
	SECTION("Random Test 50") {
		auto a = BatteryState(414.846);
		a.update_energy_remaining(833.104);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1247.95, EPSILON));
	}
	SECTION("Random Test 51") {
		auto a = BatteryState(6490.8);
		a.update_energy_remaining(300.212);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6791.01, EPSILON));
	}
	SECTION("Random Test 52") {
		auto a = BatteryState(4660.37);
		a.update_energy_remaining(-778.827);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3881.54, EPSILON));
	}
	SECTION("Random Test 53") {
		auto a = BatteryState(4211.11);
		a.update_energy_remaining(-440.72);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3770.39, EPSILON));
	}
	SECTION("Random Test 54") {
		auto a = BatteryState(5205.9);
		a.update_energy_remaining(728.284);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5934.18, EPSILON));
	}
	SECTION("Random Test 55") {
		auto a = BatteryState(620.759);
		a.update_energy_remaining(947.872);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1568.63, EPSILON));
	}
	SECTION("Random Test 56") {
		auto a = BatteryState(3134.24);
		a.update_energy_remaining(187.454);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3321.7, EPSILON));
	}
	SECTION("Random Test 57") {
		auto a = BatteryState(2585.64);
		a.update_energy_remaining(43.1121);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2628.75, EPSILON));
	}
	SECTION("Random Test 58") {
		auto a = BatteryState(3885.51);
		a.update_energy_remaining(-986.934);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2898.57, EPSILON));
	}
	SECTION("Random Test 59") {
		auto a = BatteryState(6269.85);
		a.update_energy_remaining(-780.835);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5489.01, EPSILON));
	}
	SECTION("Random Test 60") {
		auto a = BatteryState(3631.56);
		a.update_energy_remaining(311.656);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3943.22, EPSILON));
	}
	SECTION("Random Test 61") {
		auto a = BatteryState(992.855);
		a.update_energy_remaining(-395.76);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(597.096, EPSILON));
	}
	SECTION("Random Test 62") {
		auto a = BatteryState(4609.08);
		a.update_energy_remaining(534.765);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5143.84, EPSILON));
	}
	SECTION("Random Test 63") {
		auto a = BatteryState(6884.14);
		a.update_energy_remaining(-298.455);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6585.68, EPSILON));
	}
	SECTION("Random Test 64") {
		auto a = BatteryState(11.7209);
		a.update_energy_remaining(-139.764);
		REQUIRE_THAT(a.get_energy_remaining(), WithinAbs(-128.043, EPSILON));
	}
	SECTION("Random Test 65") {
		auto a = BatteryState(3386.11);
		a.update_energy_remaining(-431.349);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2954.76, EPSILON));
	}
	SECTION("Random Test 66") {
		auto a = BatteryState(3384.2);
		a.update_energy_remaining(776.141);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4160.34, EPSILON));
	}
	SECTION("Random Test 67") {
		auto a = BatteryState(5383.97);
		a.update_energy_remaining(698.64);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6082.61, EPSILON));
	}
	SECTION("Random Test 68") {
		auto a = BatteryState(7266.47);
		a.update_energy_remaining(-352.514);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6913.96, EPSILON));
	}
	SECTION("Random Test 69") {
		auto a = BatteryState(6249.41);
		a.update_energy_remaining(-302.892);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5946.51, EPSILON));
	}
	SECTION("Random Test 70") {
		auto a = BatteryState(7884.33);
		a.update_energy_remaining(-595.917);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7288.41, EPSILON));
	}
	SECTION("Random Test 71") {
		auto a = BatteryState(6518.2);
		a.update_energy_remaining(605.68);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7123.88, EPSILON));
	}
	SECTION("Random Test 72") {
		auto a = BatteryState(6085.9);
		a.update_energy_remaining(668.898);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6754.8, EPSILON));
	}
	SECTION("Random Test 73") {
		auto a = BatteryState(479.582);
		a.update_energy_remaining(531.776);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1011.36, EPSILON));
	}
	SECTION("Random Test 74") {
		auto a = BatteryState(3223.58);
		a.update_energy_remaining(-837.04);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2386.54, EPSILON));
	}
	SECTION("Random Test 75") {
		auto a = BatteryState(3936.56);
		a.update_energy_remaining(6.22925);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3942.79, EPSILON));
	}
	SECTION("Random Test 76") {
		auto a = BatteryState(2512.26);
		a.update_energy_remaining(679.197);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3191.46, EPSILON));
	}
	SECTION("Random Test 77") {
		auto a = BatteryState(6091.32);
		a.update_energy_remaining(189.266);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6280.59, EPSILON));
	}
	SECTION("Random Test 78") {
		auto a = BatteryState(7331.24);
		a.update_energy_remaining(373.183);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7704.42, EPSILON));
	}
	SECTION("Random Test 79") {
		auto a = BatteryState(2362.61);
		a.update_energy_remaining(-606.284);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1756.33, EPSILON));
	}
	SECTION("Random Test 80") {
		auto a = BatteryState(1081.61);
		a.update_energy_remaining(-799.405);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(282.206, EPSILON));
	}
	SECTION("Random Test 81") {
		auto a = BatteryState(6678.72);
		a.update_energy_remaining(-106.469);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6572.25, EPSILON));
	}
	SECTION("Random Test 82") {
		auto a = BatteryState(6095.58);
		a.update_energy_remaining(-961.093);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5134.48, EPSILON));
	}
	SECTION("Random Test 83") {
		auto a = BatteryState(6681.04);
		a.update_energy_remaining(-582.98);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6098.06, EPSILON));
	}
	SECTION("Random Test 84") {
		auto a = BatteryState(4949.9);
		a.update_energy_remaining(166.858);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5116.76, EPSILON));
	}
	SECTION("Random Test 85") {
		auto a = BatteryState(3640.99);
		a.update_energy_remaining(99.6863);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3740.67, EPSILON));
	}
	SECTION("Random Test 86") {
		auto a = BatteryState(4889.92);
		a.update_energy_remaining(-232.076);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4657.84, EPSILON));
	}
	SECTION("Random Test 87") {
		auto a = BatteryState(5316.56);
		a.update_energy_remaining(713.574);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6030.14, EPSILON));
	}
	SECTION("Random Test 88") {
		auto a = BatteryState(305.568);
		a.update_energy_remaining(-758.201);
		REQUIRE_THAT(a.get_energy_remaining(), WithinAbs(-452.634, EPSILON));
	}
	SECTION("Random Test 89") {
		auto a = BatteryState(1517.36);
		a.update_energy_remaining(-825.259);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(692.106, EPSILON));
	}
	SECTION("Random Test 90") {
		auto a = BatteryState(930.133);
		a.update_energy_remaining(269.592);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1199.73, EPSILON));
	}
	SECTION("Random Test 91") {
		auto a = BatteryState(6755.33);
		a.update_energy_remaining(11.2289);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6766.56, EPSILON));
	}
	SECTION("Random Test 92") {
		auto a = BatteryState(7290.08);
		a.update_energy_remaining(-723.054);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(6567.03, EPSILON));
	}
	SECTION("Random Test 93") {
		auto a = BatteryState(2954.07);
		a.update_energy_remaining(-957.621);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1996.45, EPSILON));
	}
	SECTION("Random Test 94") {
		auto a = BatteryState(1894.79);
		a.update_energy_remaining(876.734);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(2771.53, EPSILON));
	}
	SECTION("Random Test 95") {
		auto a = BatteryState(7011.02);
		a.update_energy_remaining(208.683);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(7219.7, EPSILON));
	}
	SECTION("Random Test 96") {
		auto a = BatteryState(3173.54);
		a.update_energy_remaining(788.583);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(3962.12, EPSILON));
	}
	SECTION("Random Test 97") {
		auto a = BatteryState(1383.79);
		a.update_energy_remaining(599.307);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(1983.09, EPSILON));
	}
	SECTION("Random Test 98") {
		auto a = BatteryState(6134.25);
		a.update_energy_remaining(-825.704);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(5308.55, EPSILON));
	}
	SECTION("Random Test 99") {
		auto a = BatteryState(4654.2);
		a.update_energy_remaining(-90.5266);
		REQUIRE_THAT(a.get_energy_remaining(), WithinRel(4563.68, EPSILON));
	}
}

TEST_CASE("Battery: state_of_charge", "[Battery]") {
	SECTION("Standard Tests") {
		const auto a = Battery(100, 0.1, 100, 100);
		REQUIRE_THAT(a.state_of_charge(100), WithinRel(1.0, EPSILON));
		REQUIRE_THAT(a.state_of_charge(0), WithinAbs(0.0, EPSILON));
		REQUIRE_THAT(a.state_of_charge(50), WithinRel(0.5, EPSILON));
		const auto b = Battery(950, 0.1, 100, 100);
		REQUIRE_THAT(b.state_of_charge(100), WithinRel(0.1052631579, EPSILON));
		REQUIRE_THAT(b.state_of_charge(0), WithinAbs(0.0, EPSILON));
		REQUIRE_THAT(b.state_of_charge(50), WithinRel(0.05263157895, EPSILON));
		REQUIRE_THAT(b.state_of_charge(950), WithinRel(1.0, EPSILON));
	}

	SECTION("Random Test 0") {
		const auto a = Battery(6365.71, 0.76229, 136.374, 160.555);
		const double result = a.state_of_charge(4435.06);
		constexpr double expected = 0.696711;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 1") {
		const auto a = Battery(777.737, 0.92556, 109.541, 150.595);
		const double result = a.state_of_charge(952.484);
		constexpr double expected = 1.22469;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 2") {
		const auto a = Battery(7263.67, 0.42446, 119.888, 141.938);
		const double result = a.state_of_charge(4053.93);
		constexpr double expected = 0.558111;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 3") {
		const auto a = Battery(4949.87, 0.185452, 109.856, 167.701);
		const double result = a.state_of_charge(5415.93);
		constexpr double expected = 1.09416;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 4") {
		const auto a = Battery(3513.78, 0.75512, 97.0248, 147.933);
		const double result = a.state_of_charge(2907.77);
		constexpr double expected = 0.827534;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 5") {
		const auto a = Battery(89.173, 0.189518, 129.179, 153.765);
		const double result = a.state_of_charge(245.044);
		constexpr double expected = 2.74796;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 6") {
		const auto a = Battery(2376.87, 0.998303, 142.929, 167.276);
		const double result = a.state_of_charge(5499.94);
		constexpr double expected = 2.31394;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 7") {
		const auto a = Battery(7737.67, 0.616942, 88.3611, 168.193);
		const double result = a.state_of_charge(7406.23);
		constexpr double expected = 0.957165;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 8") {
		const auto a = Battery(1039.83, 0.558796, 164.091, 167.612);
		const double result = a.state_of_charge(7114.76);
		constexpr double expected = 6.84222;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 9") {
		const auto a = Battery(4427.05, 0.747402, 91.4895, 158.372);
		const double result = a.state_of_charge(3159.61);
		constexpr double expected = 0.713705;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 10") {
		const auto a = Battery(6817.67, 0.973625, 98.2872, 144.701);
		const double result = a.state_of_charge(2524.89);
		constexpr double expected = 0.370345;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 11") {
		const auto a = Battery(3393.7, 0.390331, 102.693, 135.879);
		const double result = a.state_of_charge(7251.97);
		constexpr double expected = 2.13689;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 12") {
		const auto a = Battery(164.458, 0.672326, 73.938, 107.891);
		const double result = a.state_of_charge(3235.42);
		constexpr double expected = 19.6733;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 13") {
		const auto a = Battery(6267.78, 0.21437, 155.279, 161.676);
		const double result = a.state_of_charge(6486.41);
		constexpr double expected = 1.03488;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 14") {
		const auto a = Battery(3588.17, 0.708841, 92.5162, 98.6782);
		const double result = a.state_of_charge(1947.17);
		constexpr double expected = 0.542664;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 15") {
		const auto a = Battery(4625.34, 0.0874809, 113.734, 130.425);
		const double result = a.state_of_charge(6911.99);
		constexpr double expected = 1.49438;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 16") {
		const auto a = Battery(3375.09, 0.0619917, 77.0549, 95.5715);
		const double result = a.state_of_charge(2325.76);
		constexpr double expected = 0.689098;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 17") {
		const auto a = Battery(5305.83, 0.512747, 111.671, 140.429);
		const double result = a.state_of_charge(7959.11);
		constexpr double expected = 1.50007;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 18") {
		const auto a = Battery(1590.23, 0.154849, 87.195, 147.079);
		const double result = a.state_of_charge(7458.41);
		constexpr double expected = 4.69016;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 19") {
		const auto a = Battery(5599.79, 0.630372, 142.409, 144.136);
		const double result = a.state_of_charge(8014.54);
		constexpr double expected = 1.43122;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 20") {
		const auto a = Battery(7895.46, 0.679397, 123.304, 146.475);
		const double result = a.state_of_charge(6397.49);
		constexpr double expected = 0.810274;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 21") {
		const auto a = Battery(934.637, 0.0945352, 108.162, 133.387);
		const double result = a.state_of_charge(1980.67);
		constexpr double expected = 2.11919;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 22") {
		const auto a = Battery(1901.99, 0.0927242, 98.5975, 165.807);
		const double result = a.state_of_charge(6780.65);
		constexpr double expected = 3.56503;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 23") {
		const auto a = Battery(146.291, 0.920042, 99.0969, 162.278);
		const double result = a.state_of_charge(72.6514);
		constexpr double expected = 0.496623;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 24") {
		const auto a = Battery(2873.11, 0.464743, 120.081, 153.675);
		const double result = a.state_of_charge(4162.79);
		constexpr double expected = 1.44888;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 25") {
		const auto a = Battery(3379.54, 0.150757, 90.6919, 130.661);
		const double result = a.state_of_charge(3670.35);
		constexpr double expected = 1.08605;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 26") {
		const auto a = Battery(7596.48, 0.159616, 162.823, 162.898);
		const double result = a.state_of_charge(5981.23);
		constexpr double expected = 0.787369;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 27") {
		const auto a = Battery(3432.69, 0.676009, 83.4701, 130.97);
		const double result = a.state_of_charge(188.462);
		constexpr double expected = 0.054902;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 28") {
		const auto a = Battery(3018.51, 0.98414, 89.528, 96.3456);
		const double result = a.state_of_charge(2961.77);
		constexpr double expected = 0.981202;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 29") {
		const auto a = Battery(5384.8, 0.76724, 80.144, 139.896);
		const double result = a.state_of_charge(410.352);
		constexpr double expected = 0.0762056;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 30") {
		const auto a = Battery(5150, 0.304577, 76.668, 89.4133);
		const double result = a.state_of_charge(2838.07);
		constexpr double expected = 0.551083;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 31") {
		const auto a = Battery(5551.35, 0.587293, 89.6819, 117.828);
		const double result = a.state_of_charge(4996.81);
		constexpr double expected = 0.900107;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 32") {
		const auto a = Battery(4531.15, 0.985345, 128.206, 138.476);
		const double result = a.state_of_charge(1049.99);
		constexpr double expected = 0.231728;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 33") {
		const auto a = Battery(1138.62, 0.483468, 81.8592, 82.0154);
		const double result = a.state_of_charge(4453.32);
		constexpr double expected = 3.91115;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 34") {
		const auto a = Battery(3892.86, 0.0820218, 111.232, 135.508);
		const double result = a.state_of_charge(2365.2);
		constexpr double expected = 0.607576;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 35") {
		const auto a = Battery(1778.2, 0.447152, 93.9867, 114.979);
		const double result = a.state_of_charge(2955.37);
		constexpr double expected = 1.662;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 36") {
		const auto a = Battery(8041.21, 0.525913, 115.332, 147.924);
		const double result = a.state_of_charge(6031.64);
		constexpr double expected = 0.750091;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 37") {
		const auto a = Battery(2223.47, 0.39808, 100.637, 107.677);
		const double result = a.state_of_charge(628.944);
		constexpr double expected = 0.282866;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 38") {
		const auto a = Battery(385.195, 0.325613, 156.286, 156.822);
		const double result = a.state_of_charge(7716.69);
		constexpr double expected = 20.0332;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 39") {
		const auto a = Battery(202.192, 0.00697769, 70.4488, 92.8297);
		const double result = a.state_of_charge(6051.85);
		constexpr double expected = 29.9312;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 40") {
		const auto a = Battery(1132.56, 0.159764, 135.432, 160.32);
		const double result = a.state_of_charge(2334.5);
		constexpr double expected = 2.06126;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 41") {
		const auto a = Battery(6698.23, 0.861023, 116.717, 117.611);
		const double result = a.state_of_charge(1763.88);
		constexpr double expected = 0.263336;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 42") {
		const auto a = Battery(70.9049, 0.939117, 105.335, 141.778);
		const double result = a.state_of_charge(2990.6);
		constexpr double expected = 42.1776;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 43") {
		const auto a = Battery(5446.23, 0.137954, 99.7654, 146.6);
		const double result = a.state_of_charge(2139.31);
		constexpr double expected = 0.392806;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 44") {
		const auto a = Battery(7639.17, 0.204427, 116.41, 123.785);
		const double result = a.state_of_charge(7733.27);
		constexpr double expected = 1.01232;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 45") {
		const auto a = Battery(1907.94, 0.0982918, 109.423, 126.834);
		const double result = a.state_of_charge(5627.05);
		constexpr double expected = 2.94929;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 46") {
		const auto a = Battery(2200.22, 0.440392, 86.0258, 107.708);
		const double result = a.state_of_charge(4668.48);
		constexpr double expected = 2.12182;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 47") {
		const auto a = Battery(2235.89, 0.180471, 126.642, 156.272);
		const double result = a.state_of_charge(4243.81);
		constexpr double expected = 1.89804;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 48") {
		const auto a = Battery(6292.1, 0.319547, 90.4947, 161.043);
		const double result = a.state_of_charge(4707.38);
		constexpr double expected = 0.748142;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 49") {
		const auto a = Battery(7437.24, 0.340786, 71.6929, 160.433);
		const double result = a.state_of_charge(3172.83);
		constexpr double expected = 0.426613;
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

}

TEST_CASE("Battery: current_voltage", "[Battery]") {
	SECTION("Standard Tests") {
		const auto a = Battery(100, 0.1, 0, 100);
		REQUIRE_THAT(a.current_voltage(1.0), WithinRel(100.0, EPSILON));
		REQUIRE_THAT(a.current_voltage(0), WithinAbs(0.0, EPSILON));
		REQUIRE_THAT(a.current_voltage(0.5), WithinRel(50.0, EPSILON));
		REQUIRE_THAT(a.current_voltage(0.69420), WithinRel(69.42, EPSILON));
		const auto b = Battery(100, 0.1, 50, 100);
		REQUIRE_THAT(b.current_voltage(1.0), WithinRel(100.0, EPSILON));
		REQUIRE_THAT(b.current_voltage(0), WithinAbs(50.0, EPSILON));
		REQUIRE_THAT(b.current_voltage(0.5), WithinRel(75.0, EPSILON));
		REQUIRE_THAT(b.current_voltage(0.69420), WithinRel(84.71, EPSILON));
	}

	SECTION("Random Test 0") {
		const auto a = Battery(154.817, 0.141873, 145.991, 163.056);
		constexpr double state_of_charge = 32.3586;
		constexpr double expected = 698.203;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 1") {
		const auto a = Battery(7572.26, 0.622058, 80.8822, 138.822);
		constexpr double state_of_charge = 0.409037;
		constexpr double expected = 104.582;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 2") {
		const auto a = Battery(4328.83, 0.413043, 74.6436, 127.187);
		constexpr double state_of_charge = 1.51018;
		constexpr double expected = 153.993;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 3") {
		const auto a = Battery(6076.94, 0.518162, 86.2412, 134.977);
		constexpr double state_of_charge = 0.444284;
		constexpr double expected = 107.894;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 4") {
		const auto a = Battery(3566.36, 0.36489, 96.5674, 153.817);
		constexpr double state_of_charge = 1.45408;
		constexpr double expected = 179.813;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 5") {
		const auto a = Battery(7331.15, 0.139343, 145.809, 168.646);
		constexpr double state_of_charge = 0.439798;
		constexpr double expected = 155.853;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 6") {
		const auto a = Battery(3164.03, 0.528179, 93.6352, 143.594);
		constexpr double state_of_charge = 0.353324;
		constexpr double expected = 111.287;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 7") {
		const auto a = Battery(191.673, 0.229318, 73.5955, 128.8);
		constexpr double state_of_charge = 40.3301;
		constexpr double expected = 2300.01;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 8") {
		const auto a = Battery(559.704, 0.515812, 125.488, 139.268);
		constexpr double state_of_charge = 8.1702;
		constexpr double expected = 238.076;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 9") {
		const auto a = Battery(2008.67, 0.764568, 97.0003, 156.505);
		constexpr double state_of_charge = 2.55303;
		constexpr double expected = 248.919;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 10") {
		const auto a = Battery(4453.91, 0.331445, 95.8234, 106.088);
		constexpr double state_of_charge = 0.2793;
		constexpr double expected = 98.6902;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 11") {
		const auto a = Battery(5856.56, 0.443645, 113.481, 163.61);
		constexpr double state_of_charge = 0.290369;
		constexpr double expected = 128.037;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 12") {
		const auto a = Battery(5415.71, 0.633419, 90.6123, 90.7915);
		constexpr double state_of_charge = 0.0233253;
		constexpr double expected = 90.6164;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 13") {
		const auto a = Battery(5505.21, 0.0937054, 96.2623, 144.473);
		constexpr double state_of_charge = 0.201067;
		constexpr double expected = 105.956;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 14") {
		const auto a = Battery(1131.85, 0.521144, 94.0845, 157.834);
		constexpr double state_of_charge = 2.17371;
		constexpr double expected = 232.658;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 15") {
		const auto a = Battery(2606.89, 0.637497, 83.6728, 150.617);
		constexpr double state_of_charge = 0.859051;
		constexpr double expected = 141.182;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 16") {
		const auto a = Battery(7931.84, 0.217459, 73.9326, 163.311);
		constexpr double state_of_charge = 0.7005;
		constexpr double expected = 136.542;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 17") {
		const auto a = Battery(2078.94, 0.491985, 82.0737, 163.268);
		constexpr double state_of_charge = 2.25939;
		constexpr double expected = 265.523;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 18") {
		const auto a = Battery(4450.31, 0.472384, 140.074, 164.694);
		constexpr double state_of_charge = 0.738725;
		constexpr double expected = 158.261;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 19") {
		const auto a = Battery(4291.92, 0.627102, 101.524, 104.373);
		constexpr double state_of_charge = 1.74555;
		constexpr double expected = 106.496;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 20") {
		const auto a = Battery(7677.09, 0.0567154, 91.642, 164.348);
		constexpr double state_of_charge = 0.206924;
		constexpr double expected = 106.687;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 21") {
		const auto a = Battery(1955.86, 0.882038, 85.9202, 133.796);
		constexpr double state_of_charge = 2.89716;
		constexpr double expected = 224.624;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 22") {
		const auto a = Battery(6103.6, 0.970872, 122.504, 144.506);
		constexpr double state_of_charge = 0.763421;
		constexpr double expected = 139.301;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 23") {
		const auto a = Battery(6142.58, 0.85004, 90.0602, 135.032);
		constexpr double state_of_charge = 1.11809;
		constexpr double expected = 140.343;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 24") {
		const auto a = Battery(3086.64, 0.0960018, 106.975, 121.431);
		constexpr double state_of_charge = 2.24176;
		constexpr double expected = 139.383;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 25") {
		const auto a = Battery(5594.78, 0.251504, 123.423, 135.811);
		constexpr double state_of_charge = 1.16839;
		constexpr double expected = 137.897;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 26") {
		const auto a = Battery(4947.16, 0.488608, 148.676, 149.798);
		constexpr double state_of_charge = 0.324714;
		constexpr double expected = 149.04;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 27") {
		const auto a = Battery(2950.29, 0.775339, 108.549, 130.56);
		constexpr double state_of_charge = 1.60877;
		constexpr double expected = 143.96;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 28") {
		const auto a = Battery(5153.77, 0.503864, 130.634, 133.718);
		constexpr double state_of_charge = 1.28042;
		constexpr double expected = 134.583;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 29") {
		const auto a = Battery(7239.22, 0.85279, 149.557, 169.322);
		constexpr double state_of_charge = 0.97526;
		constexpr double expected = 168.833;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 30") {
		const auto a = Battery(7891.81, 0.771724, 83.5773, 97.3578);
		constexpr double state_of_charge = 0.531419;
		constexpr double expected = 90.9005;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 31") {
		const auto a = Battery(1767.76, 0.749682, 131.261, 153.06);
		constexpr double state_of_charge = 1.56103;
		constexpr double expected = 165.289;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 32") {
		const auto a = Battery(21.752, 0.209725, 90.1581, 106.209);
		constexpr double state_of_charge = 214.694;
		constexpr double expected = 3536.1;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 33") {
		const auto a = Battery(2025.47, 0.224482, 70.7748, 107.761);
		constexpr double state_of_charge = 3.32045;
		constexpr double expected = 193.587;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 34") {
		const auto a = Battery(4856.23, 0.731107, 76.0252, 115.018);
		constexpr double state_of_charge = 1.65308;
		constexpr double expected = 140.484;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 35") {
		const auto a = Battery(5849.86, 0.501283, 147, 162.112);
		constexpr double state_of_charge = 0.228799;
		constexpr double expected = 150.458;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 36") {
		const auto a = Battery(5955.44, 0.80884, 103.198, 113.548);
		constexpr double state_of_charge = 0.36346;
		constexpr double expected = 106.96;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 37") {
		const auto a = Battery(3449.71, 0.874221, 122.639, 142.364);
		constexpr double state_of_charge = 0.397413;
		constexpr double expected = 130.478;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 38") {
		const auto a = Battery(7860.35, 0.921062, 76.1602, 168.748);
		constexpr double state_of_charge = 0.559034;
		constexpr double expected = 127.92;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 39") {
		const auto a = Battery(6966.73, 0.205891, 75.8451, 114.478);
		constexpr double state_of_charge = 1.07675;
		constexpr double expected = 117.443;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 40") {
		const auto a = Battery(7836.88, 0.105677, 82.8897, 104.894);
		constexpr double state_of_charge = 0.69682;
		constexpr double expected = 98.2225;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 41") {
		const auto a = Battery(5233.08, 0.0812898, 151.476, 161.822);
		constexpr double state_of_charge = 1.20851;
		constexpr double expected = 163.979;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 42") {
		const auto a = Battery(7235.29, 0.253583, 117.853, 143.211);
		constexpr double state_of_charge = 0.31584;
		constexpr double expected = 125.862;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 43") {
		const auto a = Battery(5204.02, 0.062293, 148.048, 161.6);
		constexpr double state_of_charge = 0.192129;
		constexpr double expected = 150.652;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 44") {
		const auto a = Battery(6439.47, 0.972131, 99.5239, 157.781);
		constexpr double state_of_charge = 0.149761;
		constexpr double expected = 108.249;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 45") {
		const auto a = Battery(6166.24, 0.750698, 104.539, 150.969);
		constexpr double state_of_charge = 0.776731;
		constexpr double expected = 140.603;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 46") {
		const auto a = Battery(2183.85, 0.344996, 79.3728, 168.65);
		constexpr double state_of_charge = 1.58151;
		constexpr double expected = 220.565;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 47") {
		const auto a = Battery(2287.57, 0.0422151, 72.2191, 145.972);
		constexpr double state_of_charge = 2.43758;
		constexpr double expected = 251.997;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 48") {
		const auto a = Battery(5235.32, 0.871531, 108.18, 109.475);
		constexpr double state_of_charge = 1.53812;
		constexpr double expected = 110.172;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 49") {
		const auto a = Battery(2007.54, 0.397082, 75.8095, 144.053);
		constexpr double state_of_charge = 3.80959;
		constexpr double expected = 335.788;
		const double result = a.current_voltage(state_of_charge);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

}

TEST_CASE("Battery: power_loss", "[Battery]") {
	SECTION("Random Test 0") {
		const auto a = Battery(7459.41, 0.668212, 79.3748, 87.286);
		constexpr double state_of_charge = 0.55892;
		constexpr double net_power_demanded = -1257.31;
		constexpr double expected = 202.907;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 1") {
		const auto a = Battery(5903.81, 0.693532, 96.8445, 124.524);
		constexpr double state_of_charge = 0.0373011;
		constexpr double net_power_demanded = -3979.18;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE_FALSE(result.has_value());
	}

	SECTION("Random Test 2") {
		const auto a = Battery(6742.83, 0.624516, 90.3343, 147.014);
		constexpr double state_of_charge = 0.902358;
		constexpr double net_power_demanded = 4500.54;
		constexpr double expected = 499.467;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 3") {
		const auto a = Battery(7334.32, 0.529819, 76.3064, 83.749);
		constexpr double state_of_charge = 0.0503407;
		constexpr double net_power_demanded = 4371.86;
		constexpr double expected = 1015.22;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 4") {
		const auto a = Battery(7345.77, 0.598039, 123.394, 153.74);
		constexpr double state_of_charge = 0.0541589;
		constexpr double net_power_demanded = 4780.95;
		constexpr double expected = 652.092;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 5") {
		const auto a = Battery(2971.45, 0.112041, 108.084, 116.433);
		constexpr double state_of_charge = 0.637746;
		constexpr double net_power_demanded = -3669.65;
		constexpr double expected = 125.468;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 6") {
		const auto a = Battery(7498.73, 0.0255898, 77.1816, 158.826);
		constexpr double state_of_charge = 0.115718;
		constexpr double net_power_demanded = 3761.42;
		constexpr double expected = 47.0443;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 7") {
		const auto a = Battery(5062.51, 0.489719, 80.0837, 91.6258);
		constexpr double state_of_charge = 0.0226336;
		constexpr double net_power_demanded = -4742.48;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE_FALSE(result.has_value());
	}

	SECTION("Random Test 8") {
		const auto a = Battery(6024.27, 0.677412, 104.057, 121.93);
		constexpr double state_of_charge = 0.282905;
		constexpr double net_power_demanded = 3611.31;
		constexpr double expected = 537.564;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 9") {
		const auto a = Battery(3093.02, 0.424622, 98.2284, 166.137);
		constexpr double state_of_charge = 1.32263;
		constexpr double net_power_demanded = -2868.24;
		constexpr double expected = 106.242;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 10") {
		const auto a = Battery(6099.09, 0.00714042, 86.2665, 121.984);
		constexpr double state_of_charge = 1.05999;
		constexpr double net_power_demanded = -2388.01;
		constexpr double expected = 2.64866;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 11") {
		const auto a = Battery(1036.27, 0.687734, 149.291, 152.024);
		constexpr double state_of_charge = 6.0907;
		constexpr double net_power_demanded = 4554.25;
		constexpr double expected = 425.721;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 12") {
		const auto a = Battery(7890.98, 0.48647, 144.227, 159.731);
		constexpr double state_of_charge = 0.0219552;
		constexpr double net_power_demanded = 3173.27;
		constexpr double expected = 205.069;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 13") {
		const auto a = Battery(6884.71, 0.805665, 156.232, 163.088);
		constexpr double state_of_charge = 0.561977;
		constexpr double net_power_demanded = -1908.37;
		constexpr double expected = 130.716;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 14") {
		const auto a = Battery(5600.25, 0.802922, 84.7205, 101.105);
		constexpr double state_of_charge = 0.25167;
		constexpr double net_power_demanded = 639.881;
		constexpr double expected = 36.9755;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 15") {
		const auto a = Battery(129.041, 0.632612, 130.032, 159.442);
		constexpr double state_of_charge = 33.3526;
		constexpr double net_power_demanded = -1206.42;
		constexpr double expected = 0.746965;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 16") {
		const auto a = Battery(7221.59, 0.119513, 79.8055, 117.686);
		constexpr double state_of_charge = 0.0919661;
		constexpr double net_power_demanded = -3291.53;
		constexpr double expected = 211.398;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 17") {
		const auto a = Battery(220.63, 0.216933, 81.3373, 158.569);
		constexpr double state_of_charge = 32.0701;
		constexpr double net_power_demanded = 2503.83;
		constexpr double expected = 0.207781;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 18") {
		const auto a = Battery(6623.29, 0.913579, 95.0322, 113.571);
		constexpr double state_of_charge = 0.715831;
		constexpr double net_power_demanded = -1827.13;
		constexpr double expected = 379.117;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 19") {
		const auto a = Battery(5882.55, 0.461062, 93.7413, 140.52);
		constexpr double state_of_charge = 0.125668;
		constexpr double net_power_demanded = -2792.77;
		constexpr double expected = 505.364;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 20") {
		const auto a = Battery(4471.34, 0.839817, 119.56, 137.186);
		constexpr double state_of_charge = 1.66796;
		constexpr double net_power_demanded = -341.4;
		constexpr double expected = 4.52918;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 21") {
		const auto a = Battery(7168.71, 0.605921, 129.102, 145.873);
		constexpr double state_of_charge = 0.976701;
		constexpr double net_power_demanded = -2328.71;
		constexpr double expected = 180.206;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 22") {
		const auto a = Battery(4687.86, 0.358791, 139.323, 156.826);
		constexpr double state_of_charge = 0.427707;
		constexpr double net_power_demanded = 1267.37;
		constexpr double expected = 25.6666;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 23") {
		const auto a = Battery(3157.52, 0.721766, 99.7523, 152.878);
		constexpr double state_of_charge = 0.528539;
		constexpr double net_power_demanded = 3982.17;
		constexpr double expected = 527.232;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 24") {
		const auto a = Battery(3530.68, 0.561439, 118.674, 155.76);
		constexpr double state_of_charge = 0.47522;
		constexpr double net_power_demanded = 720.026;
		constexpr double expected = 15.0213;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 25") {
		const auto a = Battery(2949.98, 0.670562, 104.434, 156.138);
		constexpr double state_of_charge = 0.892403;
		constexpr double net_power_demanded = -4793.78;
		constexpr double expected = 989.036;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 26") {
		const auto a = Battery(544.119, 0.980762, 76.8696, 135.084);
		constexpr double state_of_charge = 8.84093;
		constexpr double net_power_demanded = 1821.82;
		constexpr double expected = 9.20881;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 27") {
		const auto a = Battery(419.943, 0.709601, 71.2573, 94.2225);
		constexpr double state_of_charge = 0.0806054;
		constexpr double net_power_demanded = 3353.69;
		constexpr double expected = 839.334;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 28") {
		const auto a = Battery(6615.28, 0.78411, 91.85, 97.3577);
		constexpr double state_of_charge = 0.15896;
		constexpr double net_power_demanded = -4102.78;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE_FALSE(result.has_value());
	}

	SECTION("Random Test 29") {
		const auto a = Battery(2434.18, 0.94465, 143.89, 157.504);
		constexpr double state_of_charge = 2.03543;
		constexpr double net_power_demanded = 4541.47;
		constexpr double expected = 519.046;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 30") {
		const auto a = Battery(7020.05, 0.101337, 133.233, 149.869);
		constexpr double state_of_charge = 0.969382;
		constexpr double net_power_demanded = 2005.96;
		constexpr double expected = 17.953;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 31") {
		const auto a = Battery(7783.17, 0.318876, 100.167, 103.519);
		constexpr double state_of_charge = 0.822927;
		constexpr double net_power_demanded = -1369.21;
		constexpr double expected = 61.6248;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 32") {
		const auto a = Battery(7800.25, 0.215752, 101.652, 148.881);
		constexpr double state_of_charge = 0.0949057;
		constexpr double net_power_demanded = 3307.64;
		constexpr double expected = 186.573;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 33") {
		const auto a = Battery(2436.62, 0.429391, 90.5819, 151.29);
		constexpr double state_of_charge = 2.44402;
		constexpr double net_power_demanded = 3128.79;
		constexpr double expected = 70.3434;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 34") {
		const auto a = Battery(4327.02, 0.980624, 78.6537, 160.965);
		constexpr double state_of_charge = 0.895415;
		constexpr double net_power_demanded = 4089.23;
		constexpr double expected = 533.975;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 35") {
		const auto a = Battery(3705.65, 0.381872, 148.079, 159.188);
		constexpr double state_of_charge = 1.78551;
		constexpr double net_power_demanded = 2251.25;
		constexpr double expected = 64.7504;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 36") {
		const auto a = Battery(7430.98, 0.248688, 127.118, 155.553);
		constexpr double state_of_charge = 0.22451;
		constexpr double net_power_demanded = -4990;
		constexpr double expected = 406.328;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 37") {
		const auto a = Battery(6084.85, 0.238945, 128.606, 161.8);
		constexpr double state_of_charge = 0.43074;
		constexpr double net_power_demanded = 3989.28;
		constexpr double expected = 170.621;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 38") {
		const auto a = Battery(4246.51, 0.987023, 123.674, 151.177);
		constexpr double state_of_charge = 1.73314;
		constexpr double net_power_demanded = -1726.37;
		constexpr double expected = 113.853;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 39") {
		const auto a = Battery(7506.57, 0.57428, 81.1429, 156.957);
		constexpr double state_of_charge = 0.0651194;
		constexpr double net_power_demanded = -153.9;
		constexpr double expected = 1.88083;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 40") {
		const auto a = Battery(7347.92, 0.362801, 82.7425, 86.0535);
		constexpr double state_of_charge = 0.549911;
		constexpr double net_power_demanded = -4508.59;
		constexpr double expected = 2472.8;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 41") {
		const auto a = Battery(3101.25, 0.0601877, 86.8036, 166.581);
		constexpr double state_of_charge = 0.460913;
		constexpr double net_power_demanded = -1385.26;
		constexpr double expected = 7.64714;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 42") {
		const auto a = Battery(6840.48, 0.0356733, 92.1114, 117.549);
		constexpr double state_of_charge = 0.311902;
		constexpr double net_power_demanded = 123.813;
		constexpr double expected = 0.0545881;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 43") {
		const auto a = Battery(6266.42, 0.423715, 77.012, 154.639);
		constexpr double state_of_charge = 1.19502;
		constexpr double net_power_demanded = 3563.03;
		constexpr double expected = 169.304;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 44") {
		const auto a = Battery(7661.21, 0.846261, 139.933, 162.902);
		constexpr double state_of_charge = 0.000772635;
		constexpr double net_power_demanded = -2342.31;
		constexpr double expected = 302.155;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 45") {
		const auto a = Battery(4307.78, 0.64965, 91.2781, 117.87);
		constexpr double state_of_charge = 0.998362;
		constexpr double net_power_demanded = 2817.19;
		constexpr double expected = 297.17;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 46") {
		const auto a = Battery(5883.81, 0.55275, 73.6375, 146.196);
		constexpr double state_of_charge = 0.251414;
		constexpr double net_power_demanded = 3517.48;
		constexpr double expected = 569.165;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 47") {
		const auto a = Battery(2964.34, 0.225666, 77.1667, 82.4987);
		constexpr double state_of_charge = 1.26291;
		constexpr double net_power_demanded = -4576.83;
		constexpr double expected = 995.39;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 48") {
		const auto a = Battery(51.7548, 0.76373, 94.4433, 157.589);
		constexpr double state_of_charge = 16.4491;
		constexpr double net_power_demanded = -4741.23;
		constexpr double expected = 13.4469;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 49") {
		const auto a = Battery(598.351, 0.654379, 104.804, 123.386);
		constexpr double state_of_charge = 8.62453;
		constexpr double net_power_demanded = 1401.79;
		constexpr double expected = 17.8381;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 50") {
		const auto a = Battery(6539.56, 0.624422, 102.167, 120.779);
		constexpr double state_of_charge = 0.553134;
		constexpr double net_power_demanded = 729.476;
		constexpr double expected = 24.5344;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 51") {
		const auto a = Battery(3549.86, 0.552174, 108.908, 113.468);
		constexpr double state_of_charge = 1.76953;
		constexpr double net_power_demanded = -3521.47;
		constexpr double expected = 729.049;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 52") {
		const auto a = Battery(1302.32, 0.23573, 104.173, 165.204);
		constexpr double state_of_charge = 4.52268;
		constexpr double net_power_demanded = -3025.47;
		constexpr double expected = 15.0765;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 53") {
		const auto a = Battery(1468.72, 0.270854, 70.8867, 146.798);
		constexpr double state_of_charge = 2.05162;
		constexpr double net_power_demanded = 2448.27;
		constexpr double expected = 30.8196;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 54") {
		const auto a = Battery(635.829, 0.85662, 72.6708, 169.765);
		constexpr double state_of_charge = 1.56848;
		constexpr double net_power_demanded = -1210.33;
		constexpr double expected = 25.867;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 55") {
		const auto a = Battery(4838.77, 0.518237, 135.51, 150.553);
		constexpr double state_of_charge = 0.995598;
		constexpr double net_power_demanded = 1969.13;
		constexpr double expected = 81.5361;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 56") {
		const auto a = Battery(300.847, 0.10359, 144.987, 157.591);
		constexpr double state_of_charge = 4.68307;
		constexpr double net_power_demanded = 478.03;
		constexpr double expected = 0.56739;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 57") {
		const auto a = Battery(3798.83, 0.0130937, 85.999, 94.3096);
		constexpr double state_of_charge = 0.406396;
		constexpr double net_power_demanded = -873.376;
		constexpr double expected = 1.2539;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 58") {
		const auto a = Battery(3433.18, 0.516766, 127.428, 168.672);
		constexpr double state_of_charge = 0.93923;
		constexpr double net_power_demanded = 799.105;
		constexpr double expected = 11.6068;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 59") {
		const auto a = Battery(1822.31, 0.516487, 125.628, 155.042);
		constexpr double state_of_charge = 0.294109;
		constexpr double net_power_demanded = 2656.93;
		constexpr double expected = 176.27;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 60") {
		const auto a = Battery(5070.73, 0.00269388, 104.908, 144.027);
		constexpr double state_of_charge = 0.685179;
		constexpr double net_power_demanded = 2691.84;
		constexpr double expected = 1.12426;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 61") {
		const auto a = Battery(3350.99, 0.555114, 98.0478, 118.874);
		constexpr double state_of_charge = 0.787191;
		constexpr double net_power_demanded = -3884.11;
		constexpr double expected = 1018.91;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 62") {
		const auto a = Battery(1717.15, 0.252387, 75.592, 166.726);
		constexpr double state_of_charge = 2.47247;
		constexpr double net_power_demanded = -686.746;
		constexpr double expected = 1.31956;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 63") {
		const auto a = Battery(3385.74, 0.270963, 100.938, 167.394);
		constexpr double state_of_charge = 0.893956;
		constexpr double net_power_demanded = 500.702;
		constexpr double expected = 2.61457;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 64") {
		const auto a = Battery(6879.58, 0.58979, 133.671, 161.479);
		constexpr double state_of_charge = 0.549431;
		constexpr double net_power_demanded = -4970.25;
		constexpr double expected = 923.392;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 65") {
		const auto a = Battery(5175.9, 0.357671, 83.6339, 162.154);
		constexpr double state_of_charge = 0.298266;
		constexpr double net_power_demanded = -3097.36;
		constexpr double expected = 376.653;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 66") {
		const auto a = Battery(6719.27, 0.138649, 163.324, 165.94);
		constexpr double state_of_charge = 0.815716;
		constexpr double net_power_demanded = -4374.41;
		constexpr double expected = 101.46;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 67") {
		const auto a = Battery(5090.59, 0.404983, 97.8688, 163.7);
		constexpr double state_of_charge = 1.01333;
		constexpr double net_power_demanded = -4612.84;
		constexpr double expected = 371.449;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 68") {
		const auto a = Battery(7775.85, 0.612807, 120.222, 138.16);
		constexpr double state_of_charge = 0.205609;
		constexpr double net_power_demanded = -3220.4;
		constexpr double expected = 574.922;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 69") {
		const auto a = Battery(2977.25, 0.0464543, 74.5399, 164.132);
		constexpr double state_of_charge = 0.965719;
		constexpr double net_power_demanded = 3469.64;
		constexpr double expected = 21.2945;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 70") {
		const auto a = Battery(3075.62, 0.728525, 144.179, 151.644);
		constexpr double state_of_charge = 2.33687;
		constexpr double net_power_demanded = 4450.22;
		constexpr double expected = 446.958;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 71") {
		const auto a = Battery(4711.48, 0.629599, 88.4393, 103.313);
		constexpr double state_of_charge = 0.843858;
		constexpr double net_power_demanded = -1421.45;
		constexpr double expected = 153.029;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 72") {
		const auto a = Battery(5360.81, 0.42242, 102.063, 120.987);
		constexpr double state_of_charge = 0.500434;
		constexpr double net_power_demanded = -521.647;
		constexpr double expected = 9.58298;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 73") {
		const auto a = Battery(3239.75, 0.93464, 127.558, 136.788);
		constexpr double state_of_charge = 1.68379;
		constexpr double net_power_demanded = 2938.54;
		constexpr double expected = 314.317;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 74") {
		const auto a = Battery(3461.59, 0.273625, 70.1545, 89.5398);
		constexpr double state_of_charge = 1.14295;
		constexpr double net_power_demanded = 3431.1;
		constexpr double expected = 312.333;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 75") {
		const auto a = Battery(3536.71, 0.780756, 116.36, 144.623);
		constexpr double state_of_charge = 0.362844;
		constexpr double net_power_demanded = 2064.27;
		constexpr double expected = 174.014;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 76") {
		const auto a = Battery(508.016, 0.299555, 70.5604, 99.8962);
		constexpr double state_of_charge = 7.85117;
		constexpr double net_power_demanded = 733.3;
		constexpr double expected = 1.77073;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 77") {
		const auto a = Battery(1466.89, 0.268122, 86.1917, 116.912);
		constexpr double state_of_charge = 3.04544;
		constexpr double net_power_demanded = -3548.68;
		constexpr double expected = 111.153;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 78") {
		const auto a = Battery(2412.66, 0.000896408, 97.1431, 97.8911);
		constexpr double state_of_charge = 2.63835;
		constexpr double net_power_demanded = -1472.44;
		constexpr double expected = 0.197881;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 79") {
		const auto a = Battery(4589.1, 0.433051, 99.9171, 131.849);
		constexpr double state_of_charge = 0.753018;
		constexpr double net_power_demanded = -285.087;
		constexpr double expected = 2.32796;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 80") {
		const auto a = Battery(4755.94, 0.53295, 94.758, 140.526);
		constexpr double state_of_charge = 1.56474;
		constexpr double net_power_demanded = 3493.18;
		constexpr double expected = 207.821;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 81") {
		const auto a = Battery(2671.18, 0.354574, 142.977, 161.909);
		constexpr double state_of_charge = 0.0100808;
		constexpr double net_power_demanded = -4758.68;
		constexpr double expected = 473.585;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 82") {
		const auto a = Battery(2653.34, 0.715639, 119.095, 158.689);
		constexpr double state_of_charge = 1.51439;
		constexpr double net_power_demanded = 1478.19;
		constexpr double expected = 45.797;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 83") {
		const auto a = Battery(3759.3, 0.710394, 80.0392, 168.733);
		constexpr double state_of_charge = 0.488191;
		constexpr double net_power_demanded = 325.832;
		constexpr double expected = 4.81239;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 84") {
		const auto a = Battery(975.035, 0.0542572, 73.0788, 97.5994);
		constexpr double state_of_charge = 3.23893;
		constexpr double net_power_demanded = -604.066;
		constexpr double expected = 0.853724;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 85") {
		const auto a = Battery(1359.87, 0.939472, 108.298, 133.992);
		constexpr double state_of_charge = 4.26603;
		constexpr double net_power_demanded = -4667.95;
		constexpr double expected = 535.733;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 86") {
		const auto a = Battery(5538.78, 0.61677, 114.954, 159.347);
		constexpr double state_of_charge = 0.23978;
		constexpr double net_power_demanded = 4054.2;
		constexpr double expected = 495.227;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 87") {
		const auto a = Battery(2540.65, 0.90957, 165.91, 169.13);
		constexpr double state_of_charge = 0.573932;
		constexpr double net_power_demanded = -2844.34;
		constexpr double expected = 324.553;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 88") {
		const auto a = Battery(1584.76, 0.171374, 112.688, 132.358);
		constexpr double state_of_charge = 1.35624;
		constexpr double net_power_demanded = -686.717;
		constexpr double expected = 4.21219;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 89") {
		const auto a = Battery(4952.33, 0.12637, 116.91, 135.401);
		constexpr double state_of_charge = 0.372507;
		constexpr double net_power_demanded = -4038.45;
		constexpr double expected = 144.255;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 90") {
		const auto a = Battery(6858.91, 0.90766, 126.906, 154.633);
		constexpr double state_of_charge = 0.888026;
		constexpr double net_power_demanded = 676.841;
		constexpr double expected = 17.2008;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 91") {
		const auto a = Battery(642.045, 0.559106, 109.01, 144.07);
		constexpr double state_of_charge = 5.49037;
		constexpr double net_power_demanded = 3507.07;
		constexpr double expected = 72.5515;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 92") {
		const auto a = Battery(2870.36, 0.785146, 126.055, 149.334);
		constexpr double state_of_charge = 1.83454;
		constexpr double net_power_demanded = -4026.19;
		constexpr double expected = 586.582;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 93") {
		const auto a = Battery(1651.11, 0.378039, 121.663, 163.146);
		constexpr double state_of_charge = 2.9118;
		constexpr double net_power_demanded = -1873.25;
		constexpr double expected = 23.1274;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 94") {
		const auto a = Battery(7398.01, 0.94136, 83.9966, 152.416);
		constexpr double state_of_charge = 0.808073;
		constexpr double net_power_demanded = -382.911;
		constexpr double expected = 7.39187;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 95") {
		const auto a = Battery(6565.24, 0.921648, 162.521, 162.895);
		constexpr double state_of_charge = 0.204357;
		constexpr double net_power_demanded = -1061.17;
		constexpr double expected = 42.4605;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 96") {
		const auto a = Battery(2061.54, 0.165739, 140.388, 157.324);
		constexpr double state_of_charge = 0.699987;
		constexpr double net_power_demanded = 849.166;
		constexpr double expected = 5.09456;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 97") {
		const auto a = Battery(2536.69, 0.905646, 77.7051, 92.1304);
		constexpr double state_of_charge = 2.24348;
		constexpr double net_power_demanded = 3460.86;
		constexpr double expected = 608.287;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 98") {
		const auto a = Battery(681.081, 0.103117, 76.2261, 125.944);
		constexpr double state_of_charge = 8.63775;
		constexpr double net_power_demanded = -2470.31;
		constexpr double expected = 2.46581;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 99") {
		const auto a = Battery(5093.31, 0.502807, 73.582, 158.624);
		constexpr double state_of_charge = 0.958833;
		constexpr double net_power_demanded = 1217.48;
		constexpr double expected = 29.4899;
		const auto result = a.power_loss(net_power_demanded, state_of_charge);
		REQUIRE(result.has_value());
		REQUIRE_THAT(result.value(), WithinRel(expected, EPSILON));
	}

}

