#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "Array.h"
constexpr double EPSILON = 0.001; // %

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("Array: power_in", "[Array]") {
	SECTION("Standard Tests") {
		const auto a = Array(1, 100);
		REQUIRE_THAT(a.power_in(1000), WithinRel(1000.0, EPSILON));
		REQUIRE_THAT(a.power_in(0), WithinRel(0.0, EPSILON));
		REQUIRE_THAT(a.power_in(500), WithinRel(500.0, EPSILON));
		const auto b = Array(1, 50);
		REQUIRE_THAT(b.power_in(1000), WithinRel(500.0));
		REQUIRE_THAT(b.power_in(0), WithinRel(0.0));
		REQUIRE_THAT(b.power_in(500), WithinRel(250.0));
		const auto c = Array(4, 25.5);
		REQUIRE_THAT(c.power_in(1000), WithinRel(1020.0));
		REQUIRE_THAT(c.power_in(0), WithinRel(0.0));
		REQUIRE_THAT(c.power_in(500), WithinRel(510.0));
		const auto d = Array(6, 20);
		REQUIRE_THAT(d.power_in(1000), WithinRel(1200.0));
		REQUIRE_THAT(d.power_in(0), WithinRel(0.0));
		REQUIRE_THAT(d.power_in(500), WithinRel(600.0));
	}

	SECTION("Random Test 0") {
		const auto a = Array(9.94047, 95.6676);
		const double expected = 8483.06;
		const double result = a.power_in(892.032);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 1") {
		const auto a = Array(8.50873, 90.8614);
		const double expected = 1717.08;
		const double result = a.power_in(222.099);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 2") {
		const auto a = Array(4.88797, 68.4851);
		const double expected = 2783.47;
		const double result = a.power_in(831.5);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 3") {
		const auto a = Array(4.47078, 11.3729);
		const double expected = 491.834;
		const double result = a.power_in(967.308);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 4") {
		const auto a = Array(2.69031, 70.2862);
		const double expected = 1550.2;
		const double result = a.power_in(819.811);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 5") {
		const auto a = Array(9.72459, 13.2036);
		const double expected = 877.799;
		const double result = a.power_in(683.649);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 6") {
		const auto a = Array(6.1919, 58.8482);
		const double expected = 3286.51;
		const double result = a.power_in(901.941);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 7") {
		const auto a = Array(4.30084, 98.9832);
		const double expected = 1385.48;
		const double result = a.power_in(325.451);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 8") {
		const auto a = Array(7.43262, 23.7239);
		const double expected = 1735.42;
		const double result = a.power_in(984.181);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 9") {
		const auto a = Array(5.79312, 45.2162);
		const double expected = 1608.27;
		const double result = a.power_in(613.977);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 10") {
		const auto a = Array(5.07093, 86.3658);
		const double expected = 227.56;
		const double result = a.power_in(51.9596);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 11") {
		const auto a = Array(7.98309, 15.0648);
		const double expected = 137.589;
		const double result = a.power_in(114.406);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 12") {
		const auto a = Array(2.35342, 6.40661);
		const double expected = 102.557;
		const double result = a.power_in(680.203);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 13") {
		const auto a = Array(1.57754, 81.5991);
		const double expected = 826.975;
		const double result = a.power_in(642.433);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 14") {
		const auto a = Array(4.31031, 41.7318);
		const double expected = 906.557;
		const double result = a.power_in(503.988);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 15") {
		const auto a = Array(5.68478, 40.4593);
		const double expected = 2246.13;
		const double result = a.power_in(976.57);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 16") {
		const auto a = Array(5.5873, 81.3565);
		const double expected = 1465.1;
		const double result = a.power_in(322.308);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 17") {
		const auto a = Array(5.22768, 12.6739);
		const double expected = 521.557;
		const double result = a.power_in(787.197);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 18") {
		const auto a = Array(9.11442, 89.2398);
		const double expected = 147.431;
		const double result = a.power_in(18.126);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 19") {
		const auto a = Array(2.87637, 89.668);
		const double expected = 1510.36;
		const double result = a.power_in(585.597);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 20") {
		const auto a = Array(6.74795, 87.8393);
		const double expected = 4986.25;
		const double result = a.power_in(841.228);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 21") {
		const auto a = Array(7.9527, 49.9176);
		const double expected = 1332.1;
		const double result = a.power_in(335.559);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 22") {
		const auto a = Array(3.52615, 92.8706);
		const double expected = 2458.17;
		const double result = a.power_in(750.644);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 23") {
		const auto a = Array(0.421924, 94.0911);
		const double expected = 315.411;
		const double result = a.power_in(794.5);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 24") {
		const auto a = Array(1.16507, 62.6555);
		const double expected = 17.0162;
		const double result = a.power_in(23.3106);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 25") {
		const auto a = Array(4.86766, 41.9319);
		const double expected = 1599.01;
		const double result = a.power_in(783.407);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 26") {
		const auto a = Array(4.18788, 80.824);
		const double expected = 3689.3;
		const double result = a.power_in(1089.96);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 27") {
		const auto a = Array(3.40133, 13.8456);
		const double expected = 18.6474;
		const double result = a.power_in(39.5964);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 28") {
		const auto a = Array(0.129428, 85.9553);
		const double expected = 22.6027;
		const double result = a.power_in(203.17);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 29") {
		const auto a = Array(6.26468, 58.8748);
		const double expected = 84.0715;
		const double result = a.power_in(22.794);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 30") {
		const auto a = Array(1.20157, 47.1519);
		const double expected = 544.871;
		const double result = a.power_in(961.712);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 31") {
		const auto a = Array(4.13966, 29.7926);
		const double expected = 1431.26;
		const double result = a.power_in(1160.5);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 32") {
		const auto a = Array(6.4076, 47.0283);
		const double expected = 250.244;
		const double result = a.power_in(83.0443);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 33") {
		const auto a = Array(2.79357, 29.675);
		const double expected = 391.167;
		const double result = a.power_in(471.859);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 34") {
		const auto a = Array(1.41483, 67.3193);
		const double expected = 951.37;
		const double result = a.power_in(998.865);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 35") {
		const auto a = Array(1.48357, 18.7038);
		const double expected = 304.058;
		const double result = a.power_in(1095.77);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 36") {
		const auto a = Array(7.20219, 17.7438);
		const double expected = 1361.16;
		const double result = a.power_in(1065.12);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 37") {
		const auto a = Array(0.555133, 78.9961);
		const double expected = 250.693;
		const double result = a.power_in(571.662);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 38") {
		const auto a = Array(3.8918, 14.3904);
		const double expected = 49.508;
		const double result = a.power_in(88.4001);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 39") {
		const auto a = Array(9.52963, 84.5125);
		const double expected = 3040.27;
		const double result = a.power_in(377.499);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 40") {
		const auto a = Array(8.3848, 99.3211);
		const double expected = 3627.83;
		const double result = a.power_in(435.624);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 41") {
		const auto a = Array(9.30979, 23.4478);
		const double expected = 1598.35;
		const double result = a.power_in(732.199);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 42") {
		const auto a = Array(1.22406, 87.3394);
		const double expected = 699.204;
		const double result = a.power_in(654.022);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 43") {
		const auto a = Array(9.31437, 15.4175);
		const double expected = 1060.45;
		const double result = a.power_in(738.454);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 44") {
		const auto a = Array(2.07042, 95.527);
		const double expected = 2078.28;
		const double result = a.power_in(1050.8);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 45") {
		const auto a = Array(1.4745, 37.2289);
		const double expected = 558.53;
		const double result = a.power_in(1017.47);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 46") {
		const auto a = Array(9.34975, 11.2353);
		const double expected = 579.646;
		const double result = a.power_in(551.796);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 47") {
		const auto a = Array(9.90425, 8.17487);
		const double expected = 445.571;
		const double result = a.power_in(550.319);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 48") {
		const auto a = Array(4.9979, 59.4918);
		const double expected = 1186.9;
		const double result = a.power_in(399.18);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 49") {
		const auto a = Array(2.87514, 67.2565);
		const double expected = 1552.75;
		const double result = a.power_in(802.989);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 50") {
		const auto a = Array(5.19913, 30.4012);
		const double expected = 295.684;
		const double result = a.power_in(187.071);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 51") {
		const auto a = Array(7.21997, 73.3998);
		const double expected = 1818.57;
		const double result = a.power_in(343.163);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 52") {
		const auto a = Array(6.92386, 38.2388);
		const double expected = 518.408;
		const double result = a.power_in(195.803);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 53") {
		const auto a = Array(7.08977, 76.3986);
		const double expected = 3819.44;
		const double result = a.power_in(705.15);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 54") {
		const auto a = Array(1.36504, 7.17491);
		const double expected = 74.4912;
		const double result = a.power_in(760.577);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 55") {
		const auto a = Array(3.17192, 75.3549);
		const double expected = 1969.97;
		const double result = a.power_in(824.187);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 56") {
		const auto a = Array(5.42645, 69.2126);
		const double expected = 3170.22;
		const double result = a.power_in(844.091);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 57") {
		const auto a = Array(6.5464, 44.9549);
		const double expected = 3127.97;
		const double result = a.power_in(1062.88);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 58") {
		const auto a = Array(3.5706, 69.932);
		const double expected = 2380.39;
		const double result = a.power_in(953.301);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 59") {
		const auto a = Array(8.42287, 93.6116);
		const double expected = 3940.77;
		const double result = a.power_in(499.795);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 60") {
		const auto a = Array(1.38813, 69.3182);
		const double expected = 939.417;
		const double result = a.power_in(976.296);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 61") {
		const auto a = Array(4.56315, 93.8102);
		const double expected = 1470.75;
		const double result = a.power_in(343.576);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 62") {
		const auto a = Array(9.06437, 81.0262);
		const double expected = 4015.72;
		const double result = a.power_in(546.764);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 63") {
		const auto a = Array(0.380459, 91.7586);
		const double expected = 381.303;
		const double result = a.power_in(1092.23);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 64") {
		const auto a = Array(3.49669, 67.2371);
		const double expected = 1501.11;
		const double result = a.power_in(638.479);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 65") {
		const auto a = Array(1.81467, 85.7187);
		const double expected = 1830.92;
		const double result = a.power_in(1177.06);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 66") {
		const auto a = Array(4.11742, 38.6189);
		const double expected = 1792.97;
		const double result = a.power_in(1127.58);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 67") {
		const auto a = Array(0.211972, 46.785);
		const double expected = 38.3422;
		const double result = a.power_in(386.626);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 68") {
		const auto a = Array(9.38286, 32.8651);
		const double expected = 371.67;
		const double result = a.power_in(120.528);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 69") {
		const auto a = Array(2.26083, 66.9064);
		const double expected = 1741;
		const double result = a.power_in(1150.97);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 70") {
		const auto a = Array(9.74741, 78.6395);
		const double expected = 2884.94;
		const double result = a.power_in(376.362);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 71") {
		const auto a = Array(3.48826, 92.8709);
		const double expected = 2751.59;
		const double result = a.power_in(849.366);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 72") {
		const auto a = Array(5.51829, 38.8607);
		const double expected = 2415.68;
		const double result = a.power_in(1126.48);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 73") {
		const auto a = Array(5.48114, 71.0976);
		const double expected = 1317.14;
		const double result = a.power_in(337.991);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 74") {
		const auto a = Array(5.39014, 20.7312);
		const double expected = 152.741;
		const double result = a.power_in(136.688);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 75") {
		const auto a = Array(1.95247, 17.4923);
		const double expected = 298.833;
		const double result = a.power_in(874.979);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 76") {
		const auto a = Array(3.73097, 58.3355);
		const double expected = 967.925;
		const double result = a.power_in(444.72);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 77") {
		const auto a = Array(3.93931, 75.5491);
		const double expected = 2917.66;
		const double result = a.power_in(980.361);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 78") {
		const auto a = Array(8.50292, 65.1481);
		const double expected = 5708.68;
		const double result = a.power_in(1030.54);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 79") {
		const auto a = Array(4.90387, 17.9726);
		const double expected = 1035.97;
		const double result = a.power_in(1175.43);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 80") {
		const auto a = Array(7.82263, 84.151);
		const double expected = 7476.8;
		const double result = a.power_in(1135.81);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 81") {
		const auto a = Array(9.49581, 62.2787);
		const double expected = 6732.48;
		const double result = a.power_in(1138.42);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 82") {
		const auto a = Array(8.99804, 33.0982);
		const double expected = 1399.87;
		const double result = a.power_in(470.04);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 83") {
		const auto a = Array(9.44213, 56.7061);
		const double expected = 3753.44;
		const double result = a.power_in(701.019);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 84") {
		const auto a = Array(4.41089, 62.7352);
		const double expected = 2000.83;
		const double result = a.power_in(723.056);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 85") {
		const auto a = Array(9.69872, 17.1651);
		const double expected = 1032.6;
		const double result = a.power_in(620.258);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 86") {
		const auto a = Array(4.99998, 4.96155);
		const double expected = 28.3377;
		const double result = a.power_in(114.23);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 87") {
		const auto a = Array(5.38579, 99.2406);
		const double expected = 2015.07;
		const double result = a.power_in(377.009);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 88") {
		const auto a = Array(9.20841, 80.0869);
		const double expected = 7870.49;
		const double result = a.power_in(1067.22);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 89") {
		const auto a = Array(4.9204, 55.3152);
		const double expected = 3151.33;
		const double result = a.power_in(1157.84);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 90") {
		const auto a = Array(5.73287, 84.386);
		const double expected = 2615.05;
		const double result = a.power_in(540.551);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 91") {
		const auto a = Array(1.41203, 1.63517);
		const double expected = 17.1701;
		const double result = a.power_in(743.648);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 92") {
		const auto a = Array(5.62504, 15.3584);
		const double expected = 959.373;
		const double result = a.power_in(1110.49);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 93") {
		const auto a = Array(1.0372, 1.19156);
		const double expected = 10.7428;
		const double result = a.power_in(869.241);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 94") {
		const auto a = Array(8.01616, 86.8626);
		const double expected = 6081.07;
		const double result = a.power_in(873.335);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 95") {
		const auto a = Array(4.95849, 19.9871);
		const double expected = 705.931;
		const double result = a.power_in(712.301);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 96") {
		const auto a = Array(2.82842, 44.8829);
		const double expected = 908.002;
		const double result = a.power_in(715.257);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 97") {
		const auto a = Array(0.954864, 6.11249);
		const double expected = 35.4525;
		const double result = a.power_in(607.417);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 98") {
		const auto a = Array(8.91322, 97.6373);
		const double expected = 9603.28;
		const double result = a.power_in(1103.49);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

	SECTION("Random Test 99") {
		const auto a = Array(7.8824, 47.152);
		const double expected = 1582.65;
		const double result = a.power_in(425.821);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}

}

