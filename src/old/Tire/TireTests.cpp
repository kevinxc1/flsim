#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "Tire.h"

constexpr double EPSILON = 0.001; // %

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("Tire: rolling_resistance", "[Tire]") {
	SECTION("Random Test 0") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.08369,
			.beta = 8.1265,
			.a = -7.48781,
			.b = 9.7679e-06,
			.c = 0.868834,
		};
		constexpr double pressure_at_stc = 131.792;
		constexpr double tire_load = 5740.22;
		constexpr double vehicle_speed = 77.0551;
		constexpr double expected = 6.83841e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 1") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.0533325,
			.beta = 3.7102,
			.a = 6.33553,
			.b = 2.84052e-06,
			.c = -0.603342,
		};
		constexpr double pressure_at_stc = 157.453;
		constexpr double tire_load = 568.231;
		constexpr double vehicle_speed = 57.239;
		constexpr double expected = -3.24406e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 2") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.91551,
			.beta = 8.68152,
			.a = 1.77167,
			.b = -4.59968e-06,
			.c = -0.903096,
		};
		constexpr double pressure_at_stc = 170.008;
		constexpr double tire_load = 7320.39;
		constexpr double vehicle_speed = 15.0325;
		constexpr double expected = -1.21996e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 3") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.95583,
			.beta = 6.27426,
			.a = 7.93755,
			.b = 7.00254e-06,
			.c = 0.894431,
		};
		constexpr double pressure_at_stc = 104.723;
		constexpr double tire_load = 7136.08;
		constexpr double vehicle_speed = 58.7222;
		constexpr double expected = 6.76861e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 4") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.75428,
			.beta = -4.95377,
			.a = 4.55093,
			.b = 9.96683e-07,
			.c = -0.0593301,
		};
		constexpr double pressure_at_stc = 173.7;
		constexpr double tire_load = 5127.9;
		constexpr double vehicle_speed = 12.0009;
		constexpr double expected = -3.01216e-23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 5") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.68708,
			.beta = -5.05005,
			.a = -0.00251466,
			.b = -1.25694e-06,
			.c = -0.546843,
		};
		constexpr double pressure_at_stc = 150.228;
		constexpr double tire_load = 9249.75;
		constexpr double vehicle_speed = 75.7547;
		constexpr double expected = -7.03514e-33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 6") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.94275,
			.beta = 2.74369,
			.a = 2.56366,
			.b = -8.73356e-06,
			.c = -0.320249,
		};
		constexpr double pressure_at_stc = 123.792;
		constexpr double tire_load = 1389.97;
		constexpr double vehicle_speed = 25.3152;
		constexpr double expected = -3.77499e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 7") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.843506,
			.beta = 7.36155,
			.a = 3.90435,
			.b = 1.4031e-06,
			.c = 0.690845,
		};
		constexpr double pressure_at_stc = 161.126;
		constexpr double tire_load = 8771.28;
		constexpr double vehicle_speed = 32.4431;
		constexpr double expected = 7.29849e+34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 8") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.14698,
			.beta = 8.69558,
			.a = -4.94044,
			.b = -1.09068e-06,
			.c = 0.243675,
		};
		constexpr double pressure_at_stc = 107.371;
		constexpr double tire_load = 2629.99;
		constexpr double vehicle_speed = 4.42501;
		constexpr double expected = 9.52439e+16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 9") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.19069,
			.beta = 0.640453,
			.a = -0.527287,
			.b = 6.03938e-06,
			.c = -0.949168,
		};
		constexpr double pressure_at_stc = 161.311;
		constexpr double tire_load = 3051.1;
		constexpr double vehicle_speed = 28.6873;
		constexpr double expected = -3.08055e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 10") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.83954,
			.beta = 2.68807,
			.a = 2.00441,
			.b = -8.24429e-06,
			.c = 0.174036,
		};
		constexpr double pressure_at_stc = 134.48;
		constexpr double tire_load = 2674.35;
		constexpr double vehicle_speed = 64.87;
		constexpr double expected = 7.54588e+29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 11") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.45451,
			.beta = -0.529883,
			.a = 0.756484,
			.b = 2.33531e-07,
			.c = -0.137812,
		};
		constexpr double pressure_at_stc = 197.158;
		constexpr double tire_load = 816.303;
		constexpr double vehicle_speed = 42.0565;
		constexpr double expected = -1.50938e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 12") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.49839,
			.beta = 1.79307,
			.a = 0.834331,
			.b = -4.68034e-06,
			.c = -0.306864,
		};
		constexpr double pressure_at_stc = 163.215;
		constexpr double tire_load = 2907.95;
		constexpr double vehicle_speed = 13.0622;
		constexpr double expected = -531991;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 13") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.71951,
			.beta = -7.48331,
			.a = -5.42874,
			.b = 8.5016e-06,
			.c = -0.573109,
		};
		constexpr double pressure_at_stc = 145.075;
		constexpr double tire_load = 5651.8;
		constexpr double vehicle_speed = 36.8782;
		constexpr double expected = -9.24464e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 14") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.52349,
			.beta = -9.36894,
			.a = -9.60748,
			.b = -1.50089e-07,
			.c = -0.333159,
		};
		constexpr double pressure_at_stc = 107.525;
		constexpr double tire_load = 3942;
		constexpr double vehicle_speed = 78.8421;
		constexpr double expected = -1.13874e-12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 15") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.40299,
			.beta = 4.97163,
			.a = 6.60833,
			.b = 5.1263e-06,
			.c = -0.56941,
		};
		constexpr double pressure_at_stc = 194.004;
		constexpr double tire_load = 6761.98;
		constexpr double vehicle_speed = 5.62843;
		constexpr double expected = -8.13416e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 16") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.43724,
			.beta = 0.638855,
			.a = -8.64618,
			.b = -7.74928e-06,
			.c = -0.447785,
		};
		constexpr double pressure_at_stc = 152.848;
		constexpr double tire_load = 5377.83;
		constexpr double vehicle_speed = 65.4463;
		constexpr double expected = -7.9904e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 17") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.10227,
			.beta = 0.197068,
			.a = 9.04855,
			.b = 3.71541e-06,
			.c = 0.0801227,
		};
		constexpr double pressure_at_stc = 146.675;
		constexpr double tire_load = 3051.26;
		constexpr double vehicle_speed = 40.0856;
		constexpr double expected = 33.3768;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 18") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.08624,
			.beta = 8.37711,
			.a = -0.771496,
			.b = -7.78471e-06,
			.c = 0.219415,
		};
		constexpr double pressure_at_stc = 160.562;
		constexpr double tire_load = 8625.69;
		constexpr double vehicle_speed = 76.5882;
		constexpr double expected = 1.5134e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 19") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.939647,
			.beta = -6.77297,
			.a = 4.46233,
			.b = -7.25421e-06,
			.c = 0.502716,
		};
		constexpr double pressure_at_stc = 141.334;
		constexpr double tire_load = 7555.71;
		constexpr double vehicle_speed = 98.4849;
		constexpr double expected = 3.57862e-20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 20") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.64859,
			.beta = -7.61586,
			.a = 2.42348,
			.b = -7.09387e-06,
			.c = 0.639936,
		};
		constexpr double pressure_at_stc = 115.5;
		constexpr double tire_load = 7880.06;
		constexpr double vehicle_speed = 3.91105;
		constexpr double expected = 6.1016e-40;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 21") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.867,
			.beta = 9.20237,
			.a = 7.96997,
			.b = -8.28091e-06,
			.c = -0.912037,
		};
		constexpr double pressure_at_stc = 116.074;
		constexpr double tire_load = 325.324;
		constexpr double vehicle_speed = 92.7417;
		constexpr double expected = -1.11168e+34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 22") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.63123,
			.beta = -3.80145,
			.a = -9.26797,
			.b = 3.39397e-06,
			.c = 0.0591257,
		};
		constexpr double pressure_at_stc = 173.404;
		constexpr double tire_load = 9298.26;
		constexpr double vehicle_speed = 42.5809;
		constexpr double expected = 5.35064e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 23") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.17095,
			.beta = 2.08089,
			.a = 0.426321,
			.b = -2.00945e-06,
			.c = -0.563718,
		};
		constexpr double pressure_at_stc = 141.592;
		constexpr double tire_load = 940.672;
		constexpr double vehicle_speed = 60.9281;
		constexpr double expected = -1.10828e-07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 24") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.18455,
			.beta = 5.62035,
			.a = 8.11743,
			.b = -1.87501e-06,
			.c = 0.313308,
		};
		constexpr double pressure_at_stc = 132.709;
		constexpr double tire_load = 1207.98;
		constexpr double vehicle_speed = 22.0994;
		constexpr double expected = 1.27848e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 25") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.68601,
			.beta = -2.18515,
			.a = -5.687,
			.b = 5.65739e-06,
			.c = 0.330484,
		};
		constexpr double pressure_at_stc = 187.083;
		constexpr double tire_load = 5790.72;
		constexpr double vehicle_speed = 76.1327;
		constexpr double expected = 1.17474e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 26") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.25078,
			.beta = -6.32259,
			.a = -6.86828,
			.b = 6.81482e-06,
			.c = -0.0627219,
		};
		constexpr double pressure_at_stc = 186.895;
		constexpr double tire_load = 7803.6;
		constexpr double vehicle_speed = 62.9469;
		constexpr double expected = -2.6832e-38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 27") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.6689,
			.beta = 8.0855,
			.a = -5.56778,
			.b = 1.51625e-06,
			.c = -0.695034,
		};
		constexpr double pressure_at_stc = 174.061;
		constexpr double tire_load = 4068.81;
		constexpr double vehicle_speed = 52.4515;
		constexpr double expected = -7.52046e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 28") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.46898,
			.beta = -9.36656,
			.a = -1.4515,
			.b = -3.9743e-06,
			.c = 0.911232,
		};
		constexpr double pressure_at_stc = 176.18;
		constexpr double tire_load = 7471.86;
		constexpr double vehicle_speed = 74.4424;
		constexpr double expected = 1.1593e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 29") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.38791,
			.beta = -1.22295,
			.a = 9.16146,
			.b = 7.65876e-06,
			.c = -0.463127,
		};
		constexpr double pressure_at_stc = 117.163;
		constexpr double tire_load = 9531.69;
		constexpr double vehicle_speed = 79.2873;
		constexpr double expected = -5.02832e-08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 30") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.952135,
			.beta = -6.33002,
			.a = 5.80457,
			.b = -8.56905e-06,
			.c = 0.538751,
		};
		constexpr double pressure_at_stc = 158.167;
		constexpr double tire_load = 8124.8;
		constexpr double vehicle_speed = 57.5274;
		constexpr double expected = 3.31736e-23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 31") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.11607,
			.beta = -2.25211,
			.a = -6.64697,
			.b = 1.67181e-06,
			.c = -0.128737,
		};
		constexpr double pressure_at_stc = 193.924;
		constexpr double tire_load = 3455.43;
		constexpr double vehicle_speed = 10.7208;
		constexpr double expected = -7.85333e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 32") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.648559,
			.beta = -9.21017,
			.a = -8.60037,
			.b = 4.49982e-06,
			.c = -0.60756,
		};
		constexpr double pressure_at_stc = 196.339;
		constexpr double tire_load = 8851.31;
		constexpr double vehicle_speed = 99.8751;
		constexpr double expected = -1.13609e-33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 33") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.65508,
			.beta = -8.9445,
			.a = -0.99306,
			.b = -3.80612e-06,
			.c = 0.871426,
		};
		constexpr double pressure_at_stc = 164.426;
		constexpr double tire_load = 999.976;
		constexpr double vehicle_speed = 72.7984;
		constexpr double expected = 1.57107e-37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 34") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.930696,
			.beta = 5.33118,
			.a = 0.174986,
			.b = -9.38356e-07,
			.c = -0.396875,
		};
		constexpr double pressure_at_stc = 112.656;
		constexpr double tire_load = 2616.47;
		constexpr double vehicle_speed = 58.6227;
		constexpr double expected = -2.38462e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 35") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.557,
			.beta = 3.84432,
			.a = -8.06519,
			.b = -6.04309e-06,
			.c = -0.414417,
		};
		constexpr double pressure_at_stc = 152.506;
		constexpr double tire_load = 4824.98;
		constexpr double vehicle_speed = 0.58945;
		constexpr double expected = -1.95025e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 36") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.88224,
			.beta = -8.11299,
			.a = -5.05502,
			.b = 1.75974e-06,
			.c = 0.700785,
		};
		constexpr double pressure_at_stc = 176.587;
		constexpr double tire_load = 3882.08;
		constexpr double vehicle_speed = 94.3377;
		constexpr double expected = 3.63228e-29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 37") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.60346,
			.beta = -1.93817,
			.a = -5.44081,
			.b = -7.0023e-06,
			.c = -0.722699,
		};
		constexpr double pressure_at_stc = 116.967;
		constexpr double tire_load = 2900.76;
		constexpr double vehicle_speed = 42.6386;
		constexpr double expected = -1.75712e+13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 38") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.40095,
			.beta = 1.23624,
			.a = 5.5496,
			.b = 9.35679e-06,
			.c = 0.770253,
		};
		constexpr double pressure_at_stc = 112.428;
		constexpr double tire_load = 5877.77;
		constexpr double vehicle_speed = 10.8327;
		constexpr double expected = 5.69472;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 39") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.0764,
			.beta = 3.19506,
			.a = -9.05147,
			.b = -5.96909e-06,
			.c = 0.506656,
		};
		constexpr double pressure_at_stc = 193.895;
		constexpr double tire_load = 5327.09;
		constexpr double vehicle_speed = 94.246;
		constexpr double expected = 9.93538e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 40") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.8027,
			.beta = -3.93798,
			.a = -5.0038,
			.b = 1.67418e-06,
			.c = -0.0592058,
		};
		constexpr double pressure_at_stc = 103.923;
		constexpr double tire_load = 3321.47;
		constexpr double vehicle_speed = 58.8155;
		constexpr double expected = -196632;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 41") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.19617,
			.beta = -2.43424,
			.a = -6.6121,
			.b = -5.46354e-06,
			.c = -0.71154,
		};
		constexpr double pressure_at_stc = 149.098;
		constexpr double tire_load = 1406.69;
		constexpr double vehicle_speed = 66.5851;
		constexpr double expected = -1.36143e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 42") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.31576,
			.beta = 9.43945,
			.a = 2.18595,
			.b = 5.30432e-07,
			.c = -0.213213,
		};
		constexpr double pressure_at_stc = 111.132;
		constexpr double tire_load = 928.864;
		constexpr double vehicle_speed = 16.7923;
		constexpr double expected = -1.19368e+22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 43") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.18358,
			.beta = -1.48986,
			.a = -4.82887,
			.b = 9.41166e-06,
			.c = 0.291045,
		};
		constexpr double pressure_at_stc = 136.134;
		constexpr double tire_load = 3169.71;
		constexpr double vehicle_speed = 48.3797;
		constexpr double expected = 2.12377e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 44") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.40871,
			.beta = 6.31054,
			.a = 9.76124,
			.b = -6.41499e-06,
			.c = -0.922594,
		};
		constexpr double pressure_at_stc = 150.635;
		constexpr double tire_load = 5946.73;
		constexpr double vehicle_speed = 29.1259;
		constexpr double expected = -1.66588e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 45") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.8928,
			.beta = 9.73914,
			.a = -2.57344,
			.b = -6.46263e-06,
			.c = -0.430031,
		};
		constexpr double pressure_at_stc = 110.65;
		constexpr double tire_load = 7749.01;
		constexpr double vehicle_speed = 50.5683;
		constexpr double expected = -8.77607e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 46") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.7183,
			.beta = 2.00355,
			.a = 4.88935,
			.b = -6.6187e-06,
			.c = 0.162534,
		};
		constexpr double pressure_at_stc = 154.351;
		constexpr double tire_load = 8731.03;
		constexpr double vehicle_speed = 16.3519;
		constexpr double expected = 2.57673e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 47") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.34249,
			.beta = 8.28776,
			.a = 0.522561,
			.b = -3.57659e-06,
			.c = -0.213935,
		};
		constexpr double pressure_at_stc = 126.024;
		constexpr double tire_load = 6425.18;
		constexpr double vehicle_speed = 26.2168;
		constexpr double expected = -1.82518e+50;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 48") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.14671,
			.beta = -7.93985,
			.a = 2.01336,
			.b = 2.91412e-06,
			.c = -0.811942,
		};
		constexpr double pressure_at_stc = 104.957;
		constexpr double tire_load = 4908.32;
		constexpr double vehicle_speed = 63.7365;
		constexpr double expected = -4.61263e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 49") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.66377,
			.beta = 1.15643,
			.a = -5.12782,
			.b = -5.44007e-07,
			.c = 0.918364,
		};
		constexpr double pressure_at_stc = 160.068;
		constexpr double tire_load = 2380.73;
		constexpr double vehicle_speed = 39.5698;
		constexpr double expected = 1.91364e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 50") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.14633,
			.beta = -3.13035,
			.a = 1.895,
			.b = -1.53126e-06,
			.c = 0.860458,
		};
		constexpr double pressure_at_stc = 187.686;
		constexpr double tire_load = 5973.14;
		constexpr double vehicle_speed = 2.61976;
		constexpr double expected = 8.33058e-18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 51") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.75238,
			.beta = 6.79751,
			.a = -6.00388,
			.b = -7.18933e-06,
			.c = -0.530751,
		};
		constexpr double pressure_at_stc = 125.289;
		constexpr double tire_load = 4453.27;
		constexpr double vehicle_speed = 88.051;
		constexpr double expected = -4.95286e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 52") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.08807,
			.beta = -3.03573,
			.a = -5.56472,
			.b = 5.08314e-06,
			.c = 0.680136,
		};
		constexpr double pressure_at_stc = 158.921;
		constexpr double tire_load = 347.597;
		constexpr double vehicle_speed = 83.6382;
		constexpr double expected = 4.76507e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 53") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.965537,
			.beta = 9.69855,
			.a = -7.39558,
			.b = -3.30888e-06,
			.c = -0.260887,
		};
		constexpr double pressure_at_stc = 156.899;
		constexpr double tire_load = 7312.18;
		constexpr double vehicle_speed = 41.403;
		constexpr double expected = -1.31638e+39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 54") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.07762,
			.beta = -4.85999,
			.a = 1.10176,
			.b = -3.32819e-06,
			.c = 0.644854,
		};
		constexpr double pressure_at_stc = 153.542;
		constexpr double tire_load = 2512;
		constexpr double vehicle_speed = 14.993;
		constexpr double expected = 2.47814e-16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 55") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.97305,
			.beta = 3.77527,
			.a = 3.42115,
			.b = 3.26148e-06,
			.c = 0.754063,
		};
		constexpr double pressure_at_stc = 170.308;
		constexpr double tire_load = 9002.17;
		constexpr double vehicle_speed = 87.8235;
		constexpr double expected = 17679.9;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 56") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.517075,
			.beta = -4.35347,
			.a = -5.17531,
			.b = 9.1927e-06,
			.c = -0.243031,
		};
		constexpr double pressure_at_stc = 116.464;
		constexpr double tire_load = 126.179;
		constexpr double vehicle_speed = 93.6167;
		constexpr double expected = -1.68303e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 57") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.39256,
			.beta = 8.48051,
			.a = 2.04971,
			.b = -8.33494e-06,
			.c = 0.997641,
		};
		constexpr double pressure_at_stc = 171.819;
		constexpr double tire_load = 814.063;
		constexpr double vehicle_speed = 82.1379;
		constexpr double expected = 2.1713e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 58") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.9192,
			.beta = -9.18787,
			.a = 8.6562,
			.b = 5.89547e-06,
			.c = 0.904916,
		};
		constexpr double pressure_at_stc = 181.51;
		constexpr double tire_load = 6889.31;
		constexpr double vehicle_speed = 26.3035;
		constexpr double expected = 1.12424e-38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 59") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.03695,
			.beta = 4.23481,
			.a = -8.39194,
			.b = -6.64281e-06,
			.c = -0.457254,
		};
		constexpr double pressure_at_stc = 131.19;
		constexpr double tire_load = 6733.09;
		constexpr double vehicle_speed = 40.5704;
		constexpr double expected = -198501;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 60") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.721,
			.beta = -8.00636,
			.a = 3.27578,
			.b = -5.32106e-06,
			.c = -0.959984,
		};
		constexpr double pressure_at_stc = 115.817;
		constexpr double tire_load = 7672.16;
		constexpr double vehicle_speed = 7.71949;
		constexpr double expected = -5.83337e-47;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 61") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.92718,
			.beta = -6.74854,
			.a = 0.789893,
			.b = -1.95139e-06,
			.c = -0.998247,
		};
		constexpr double pressure_at_stc = 131.04;
		constexpr double tire_load = 6945.62;
		constexpr double vehicle_speed = 23.4418;
		constexpr double expected = -1.81218e-37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 62") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.13072,
			.beta = 8.20984,
			.a = -1.93734,
			.b = -5.12055e-06,
			.c = -0.732488,
		};
		constexpr double pressure_at_stc = 141.686;
		constexpr double tire_load = 6237.37;
		constexpr double vehicle_speed = 37.9633;
		constexpr double expected = -5.31003e+37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 63") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.47684,
			.beta = 3.5651,
			.a = -7.52607,
			.b = 9.92786e-06,
			.c = -0.661524,
		};
		constexpr double pressure_at_stc = 151.265;
		constexpr double tire_load = 7346.61;
		constexpr double vehicle_speed = 99.3079;
		constexpr double expected = -1.35379e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 64") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.1939,
			.beta = -4.83222,
			.a = -8.92552,
			.b = -8.11053e-06,
			.c = -0.0319825,
		};
		constexpr double pressure_at_stc = 118.081;
		constexpr double tire_load = 621.501;
		constexpr double vehicle_speed = 53.4686;
		constexpr double expected = -1.33235e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 65") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.05812,
			.beta = -8.78783,
			.a = 8.07311,
			.b = 4.982e-06,
			.c = 0.107727,
		};
		constexpr double pressure_at_stc = 101.933;
		constexpr double tire_load = 3498.41;
		constexpr double vehicle_speed = 9.26969;
		constexpr double expected = 1.30135e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 66") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.223679,
			.beta = 5.01546,
			.a = -5.65565,
			.b = 8.07568e-06,
			.c = 0.0921112,
		};
		constexpr double pressure_at_stc = 114.665;
		constexpr double tire_load = 5056.12;
		constexpr double vehicle_speed = 80.0651;
		constexpr double expected = 9.98099e+21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 67") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.84557,
			.beta = -0.913191,
			.a = -8.78635,
			.b = 2.88e-07,
			.c = 0.330787,
		};
		constexpr double pressure_at_stc = 139.415;
		constexpr double tire_load = 8142.52;
		constexpr double vehicle_speed = 68.135;
		constexpr double expected = 4.12551e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 68") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.8197,
			.beta = 9.83986,
			.a = 1.43792,
			.b = 4.29739e-06,
			.c = -0.512441,
		};
		constexpr double pressure_at_stc = 183.747;
		constexpr double tire_load = 956.948;
		constexpr double vehicle_speed = 42.6504;
		constexpr double expected = -1.965e+29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 69") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.946455,
			.beta = -0.0563203,
			.a = -1.86889,
			.b = -3.19426e-06,
			.c = 0.538298,
		};
		constexpr double pressure_at_stc = 164.143;
		constexpr double tire_load = 9864.56;
		constexpr double vehicle_speed = 78.5436;
		constexpr double expected = 3.20252e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 70") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.31097,
			.beta = 8.22797,
			.a = -6.13308,
			.b = -5.72299e-06,
			.c = 0.785397,
		};
		constexpr double pressure_at_stc = 195.084;
		constexpr double tire_load = 6318.28;
		constexpr double vehicle_speed = 41.0956;
		constexpr double expected = 8.38442e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 71") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.87418,
			.beta = -5.52611,
			.a = -1.45626,
			.b = 4.89942e-06,
			.c = 0.865552,
		};
		constexpr double pressure_at_stc = 112.382;
		constexpr double tire_load = 1834.12;
		constexpr double vehicle_speed = 52.5958;
		constexpr double expected = 402.943;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 72") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.32977,
			.beta = 9.40517,
			.a = 6.18894,
			.b = -8.86182e-06,
			.c = 0.823069,
		};
		constexpr double pressure_at_stc = 132.061;
		constexpr double tire_load = 6845.92;
		constexpr double vehicle_speed = 32.11;
		constexpr double expected = 1.13594e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 73") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.37164,
			.beta = 5.48457,
			.a = 9.63914,
			.b = -9.53908e-06,
			.c = 0.746916,
		};
		constexpr double pressure_at_stc = 199.45;
		constexpr double tire_load = 9655.16;
		constexpr double vehicle_speed = 58.4433;
		constexpr double expected = 66216.6;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 74") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.657958,
			.beta = 9.05119,
			.a = -6.89446,
			.b = -3.5321e-06,
			.c = 0.407147,
		};
		constexpr double pressure_at_stc = 135.932;
		constexpr double tire_load = 3375.6;
		constexpr double vehicle_speed = 19.5194;
		constexpr double expected = 4.37735e+36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 75") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.16542,
			.beta = 5.13077,
			.a = 6.54335,
			.b = 1.13802e-06,
			.c = -0.00933867,
		};
		constexpr double pressure_at_stc = 151.373;
		constexpr double tire_load = 3348.46;
		constexpr double vehicle_speed = 72.5578;
		constexpr double expected = -1.39898e+32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 76") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.0519,
			.beta = 2.31359,
			.a = -9.11175,
			.b = -4.48866e-07,
			.c = -0.136324,
		};
		constexpr double pressure_at_stc = 169.736;
		constexpr double tire_load = 3563.11;
		constexpr double vehicle_speed = 93.1194;
		constexpr double expected = -7.89541e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 77") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.8876,
			.beta = 9.60937,
			.a = -8.70958,
			.b = 2.93843e-06,
			.c = -0.111281,
		};
		constexpr double pressure_at_stc = 167.51;
		constexpr double tire_load = 5851.11;
		constexpr double vehicle_speed = 33.5672;
		constexpr double expected = -5.85817e+30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 78") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.5579,
			.beta = 8.12365,
			.a = -3.88529,
			.b = -3.75384e-06,
			.c = 0.362935,
		};
		constexpr double pressure_at_stc = 197.513;
		constexpr double tire_load = 2837.54;
		constexpr double vehicle_speed = 92.9133;
		constexpr double expected = 7.94936e+19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 79") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.25517,
			.beta = -7.41571,
			.a = -6.17017,
			.b = 5.54444e-06,
			.c = -0.371999,
		};
		constexpr double pressure_at_stc = 124.658;
		constexpr double tire_load = 3178.19;
		constexpr double vehicle_speed = 81.305;
		constexpr double expected = -2.12575e-37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 80") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.65049,
			.beta = 9.01378,
			.a = -5.47208,
			.b = 4.77299e-06,
			.c = -0.682068,
		};
		constexpr double pressure_at_stc = 149.364;
		constexpr double tire_load = 7480.45;
		constexpr double vehicle_speed = 84.3294;
		constexpr double expected = -1.8086e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 81") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.39848,
			.beta = -3.66069,
			.a = -2.10029,
			.b = 6.69782e-06,
			.c = -0.444562,
		};
		constexpr double pressure_at_stc = 118.398;
		constexpr double tire_load = 1726.65;
		constexpr double vehicle_speed = 75.2163;
		constexpr double expected = -2.95043e-19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 82") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.45551,
			.beta = -3.96333,
			.a = 8.23952,
			.b = 7.23473e-06,
			.c = 0.00360431,
		};
		constexpr double pressure_at_stc = 120.465;
		constexpr double tire_load = 9978.14;
		constexpr double vehicle_speed = 83.4224;
		constexpr double expected = 1.27737;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 83") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.425,
			.beta = -4.923,
			.a = -8.98145,
			.b = 5.13948e-06,
			.c = 0.911031,
		};
		constexpr double pressure_at_stc = 169.106;
		constexpr double tire_load = 4340.7;
		constexpr double vehicle_speed = 87.3955;
		constexpr double expected = 3.18633e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 84") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.46174,
			.beta = -3.93734,
			.a = -1.93824,
			.b = 8.69777e-07,
			.c = 0.556994,
		};
		constexpr double pressure_at_stc = 168.64;
		constexpr double tire_load = 9247.99;
		constexpr double vehicle_speed = 82.6792;
		constexpr double expected = 17.4032;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 85") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.34602,
			.beta = -8.23219,
			.a = 3.61489,
			.b = -8.67265e-06,
			.c = 0.116535,
		};
		constexpr double pressure_at_stc = 132.557;
		constexpr double tire_load = 5445;
		constexpr double vehicle_speed = 41.1193;
		constexpr double expected = 1.52589e-41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 86") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.56125,
			.beta = 4.43365,
			.a = -0.202252,
			.b = -6.9286e-06,
			.c = -0.540747,
		};
		constexpr double pressure_at_stc = 105.284;
		constexpr double tire_load = 1039.93;
		constexpr double vehicle_speed = 91.0455;
		constexpr double expected = -6.70649;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 87") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.95572,
			.beta = 0.462289,
			.a = -1.36273,
			.b = 6.002e-06,
			.c = -0.922499,
		};
		constexpr double pressure_at_stc = 177.196;
		constexpr double tire_load = 2125.73;
		constexpr double vehicle_speed = 43.6391;
		constexpr double expected = -0.00100315;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 88") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.8793,
			.beta = -5.74051,
			.a = -3.01715,
			.b = 6.48929e-06,
			.c = -0.852656,
		};
		constexpr double pressure_at_stc = 189.302;
		constexpr double tire_load = 5325.13;
		constexpr double vehicle_speed = 72.8817;
		constexpr double expected = -2.7241e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 89") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.592,
			.beta = -6.10014,
			.a = 7.92211,
			.b = -9.88776e-06,
			.c = 0.1955,
		};
		constexpr double pressure_at_stc = 188.432;
		constexpr double tire_load = 5857.51;
		constexpr double vehicle_speed = 53.0473;
		constexpr double expected = 2.64521e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 90") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.207072,
			.beta = -1.19824,
			.a = 6.36707,
			.b = -7.62995e-06,
			.c = -0.942479,
		};
		constexpr double pressure_at_stc = 163.756;
		constexpr double tire_load = 2863.5;
		constexpr double vehicle_speed = 65.3517;
		constexpr double expected = -1.30807;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 91") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.71687,
			.beta = 8.69016,
			.a = 8.06248,
			.b = -2.03657e-06,
			.c = 0.2716,
		};
		constexpr double pressure_at_stc = 137.318;
		constexpr double tire_load = 2943.37;
		constexpr double vehicle_speed = 93.3193;
		constexpr double expected = 2.00269e+38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 92") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.13453,
			.beta = -2.87244,
			.a = 4.87784,
			.b = -6.61452e-06,
			.c = 0.652629,
		};
		constexpr double pressure_at_stc = 130.124;
		constexpr double tire_load = 14.0787;
		constexpr double vehicle_speed = 62.8436;
		constexpr double expected = 1.56775e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 93") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.51162,
			.beta = 8.3994,
			.a = -5.14451,
			.b = -1.78774e-06,
			.c = -0.692327,
		};
		constexpr double pressure_at_stc = 179.529;
		constexpr double tire_load = 7177.15;
		constexpr double vehicle_speed = 33.0255;
		constexpr double expected = -3.40226e+37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 94") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.21381,
			.beta = -2.97762,
			.a = -9.11808,
			.b = 8.79671e-06,
			.c = 0.538696,
		};
		constexpr double pressure_at_stc = 131.828;
		constexpr double tire_load = 4246.52;
		constexpr double vehicle_speed = 62.6027;
		constexpr double expected = 6.41811e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 95") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.18329,
			.beta = -3.30226,
			.a = 7.83043,
			.b = 2.35564e-06,
			.c = -0.788644,
		};
		constexpr double pressure_at_stc = 138.063;
		constexpr double tire_load = 9657.49;
		constexpr double vehicle_speed = 27.6574;
		constexpr double expected = -0.0035163;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 96") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.673986,
			.beta = -5.2761,
			.a = -5.35842,
			.b = -4.83069e-06,
			.c = -0.73391,
		};
		constexpr double pressure_at_stc = 122.401;
		constexpr double tire_load = 4649.08;
		constexpr double vehicle_speed = 26.061;
		constexpr double expected = -1.13257e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 97") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.10056,
			.beta = 0.398277,
			.a = -6.97246,
			.b = -9.07607e-06,
			.c = 0.274824,
		};
		constexpr double pressure_at_stc = 190.768;
		constexpr double tire_load = 7700.64;
		constexpr double vehicle_speed = 88.2272;
		constexpr double expected = 1.72505e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 98") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.30852,
			.beta = -8.60516,
			.a = -4.97419,
			.b = -4.87207e-07,
			.c = 0.294653,
		};
		constexpr double pressure_at_stc = 165.822;
		constexpr double tire_load = 3737.39;
		constexpr double vehicle_speed = 28.7665;
		constexpr double expected = 4.28932e-33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 99") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.29412,
			.beta = 4.35977,
			.a = -6.70322,
			.b = 1.34143e-06,
			.c = -0.430046,
		};
		constexpr double pressure_at_stc = 164.673;
		constexpr double tire_load = 7180.75;
		constexpr double vehicle_speed = 44.4737;
		constexpr double expected = -2.8584e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 100") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.02798,
			.beta = -5.75747,
			.a = -4.91476,
			.b = -7.13416e-06,
			.c = 0.286534,
		};
		constexpr double pressure_at_stc = 199.185;
		constexpr double tire_load = 5857.17;
		constexpr double vehicle_speed = 41.696;
		constexpr double expected = 9.08126e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 101") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.97329,
			.beta = 4.84493,
			.a = -8.32645,
			.b = -2.63868e-06,
			.c = -0.722448,
		};
		constexpr double pressure_at_stc = 123.011;
		constexpr double tire_load = 7933.74;
		constexpr double vehicle_speed = 60.7577;
		constexpr double expected = -1.53137e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 102") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.39207,
			.beta = 6.24216,
			.a = 1.56343,
			.b = -9.2591e-06,
			.c = -0.353572,
		};
		constexpr double pressure_at_stc = 175.742;
		constexpr double tire_load = 2768.06;
		constexpr double vehicle_speed = 9.10873;
		constexpr double expected = -4.56148e+40;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 103") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.16142,
			.beta = -7.17791,
			.a = -4.10005,
			.b = 9.06681e-06,
			.c = 0.0965543,
		};
		constexpr double pressure_at_stc = 144.633;
		constexpr double tire_load = 9596.19;
		constexpr double vehicle_speed = 59.9372;
		constexpr double expected = 2.51165e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 104") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.25311,
			.beta = -6.83101,
			.a = 8.95617,
			.b = -1.68781e-06,
			.c = -0.228139,
		};
		constexpr double pressure_at_stc = 134.558;
		constexpr double tire_load = 1948.25;
		constexpr double vehicle_speed = 44.543;
		constexpr double expected = -2.24182e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 105") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.7758,
			.beta = 2.23956,
			.a = -3.00781,
			.b = -1.39419e-06,
			.c = -0.433373,
		};
		constexpr double pressure_at_stc = 125.927;
		constexpr double tire_load = 8803.44;
		constexpr double vehicle_speed = 36.2281;
		constexpr double expected = -470.175;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 106") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.43873,
			.beta = 2.8487,
			.a = -9.58503,
			.b = 8.97068e-06,
			.c = -0.733736,
		};
		constexpr double pressure_at_stc = 187.2;
		constexpr double tire_load = 7670.54;
		constexpr double vehicle_speed = 31.2999;
		constexpr double expected = -2.02129e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 107") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.3386,
			.beta = 9.7057,
			.a = -8.14802,
			.b = -5.42304e-06,
			.c = 0.457631,
		};
		constexpr double pressure_at_stc = 150.19;
		constexpr double tire_load = 8791.68;
		constexpr double vehicle_speed = 55.7629;
		constexpr double expected = 9.75379e+51;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 108") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.51755,
			.beta = -0.31874,
			.a = 3.19188,
			.b = 5.51208e-06,
			.c = -0.516642,
		};
		constexpr double pressure_at_stc = 107.913;
		constexpr double tire_load = 4914.71;
		constexpr double vehicle_speed = 9.88745;
		constexpr double expected = -0.000330248;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 109") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.62482,
			.beta = -7.82762,
			.a = -5.30366,
			.b = 3.82844e-06,
			.c = 0.615304,
		};
		constexpr double pressure_at_stc = 112.31;
		constexpr double tire_load = 1607.29;
		constexpr double vehicle_speed = 33.269;
		constexpr double expected = 1.64187e-37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 110") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.86919,
			.beta = -5.14751,
			.a = -8.76513,
			.b = 7.08027e-06,
			.c = -0.545199,
		};
		constexpr double pressure_at_stc = 125.537;
		constexpr double tire_load = 3492.95;
		constexpr double vehicle_speed = 79.9189;
		constexpr double expected = -3.43931e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 111") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.9979,
			.beta = -7.99648,
			.a = -1.15887,
			.b = -5.57823e-06,
			.c = 0.244123,
		};
		constexpr double pressure_at_stc = 139.919;
		constexpr double tire_load = 4249.91;
		constexpr double vehicle_speed = 4.90061;
		constexpr double expected = 3.55932e-47;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 112") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.15266,
			.beta = 3.18126,
			.a = 3.43845,
			.b = 5.7184e-06,
			.c = 0.504556,
		};
		constexpr double pressure_at_stc = 192.64;
		constexpr double tire_load = 8417.28;
		constexpr double vehicle_speed = 89.5234;
		constexpr double expected = 2.56702e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 113") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.09663,
			.beta = 9.26113,
			.a = 8.78621,
			.b = -5.63651e-06,
			.c = 0.247031,
		};
		constexpr double pressure_at_stc = 174.267;
		constexpr double tire_load = 6641.16;
		constexpr double vehicle_speed = 4.09745;
		constexpr double expected = 5.91295e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 114") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.41831,
			.beta = 4.13442,
			.a = -4.67891,
			.b = -8.87769e-07,
			.c = 0.86635,
		};
		constexpr double pressure_at_stc = 104.63;
		constexpr double tire_load = 5915.5;
		constexpr double vehicle_speed = 59.5527;
		constexpr double expected = 1.25558e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 115") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.848892,
			.beta = 5.1366,
			.a = -4.8268,
			.b = -7.40507e-06,
			.c = -0.971162,
		};
		constexpr double pressure_at_stc = 175.864;
		constexpr double tire_load = 8629.63;
		constexpr double vehicle_speed = 57.1279;
		constexpr double expected = -5.45989e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 116") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.31642,
			.beta = 4.05365,
			.a = -9.27495,
			.b = 7.01807e-06,
			.c = 0.743653,
		};
		constexpr double pressure_at_stc = 192.107;
		constexpr double tire_load = 3478.89;
		constexpr double vehicle_speed = 96.8436;
		constexpr double expected = 402.218;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 117") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.22939,
			.beta = -1.02517,
			.a = 8.65308,
			.b = -3.12371e-06,
			.c = -0.397331,
		};
		constexpr double pressure_at_stc = 113.932;
		constexpr double tire_load = 7228.19;
		constexpr double vehicle_speed = 7.3306;
		constexpr double expected = -3.09379e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 118") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.37821,
			.beta = -0.849038,
			.a = 1.00201,
			.b = 2.61251e-06,
			.c = 0.524716,
		};
		constexpr double pressure_at_stc = 165.878;
		constexpr double tire_load = 6291.14;
		constexpr double vehicle_speed = 7.5046;
		constexpr double expected = 1.20209e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 119") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.04039,
			.beta = -8.5904,
			.a = -2.37739,
			.b = -6.63554e-06,
			.c = -0.900298,
		};
		constexpr double pressure_at_stc = 173.583;
		constexpr double tire_load = 1158.34;
		constexpr double vehicle_speed = 53.4467;
		constexpr double expected = -5.38009e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 120") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.16824,
			.beta = -2.7169,
			.a = -4.40582,
			.b = -7.09575e-07,
			.c = -0.0691618,
		};
		constexpr double pressure_at_stc = 190.645;
		constexpr double tire_load = 9613.23;
		constexpr double vehicle_speed = 64.3977;
		constexpr double expected = -6.52638e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 121") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.880553,
			.beta = -2.96753,
			.a = 9.80794,
			.b = -8.65311e-06,
			.c = -0.726677,
		};
		constexpr double pressure_at_stc = 130.133;
		constexpr double tire_load = 9408.09;
		constexpr double vehicle_speed = 68.0557;
		constexpr double expected = -9.68852e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 122") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.72778,
			.beta = -8.43447,
			.a = -6.41415,
			.b = -1.3961e-06,
			.c = 0.640146,
		};
		constexpr double pressure_at_stc = 134.255;
		constexpr double tire_load = 3914.15;
		constexpr double vehicle_speed = 97.4924;
		constexpr double expected = 1.09388e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 123") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.7963,
			.beta = 0.156338,
			.a = -2.5584,
			.b = 7.69594e-06,
			.c = -0.778946,
		};
		constexpr double pressure_at_stc = 153.773;
		constexpr double tire_load = 2165.99;
		constexpr double vehicle_speed = 88.9541;
		constexpr double expected = -3.46038e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 124") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.844359,
			.beta = -7.47025,
			.a = 0.625562,
			.b = 4.42511e-06,
			.c = -0.870953,
		};
		constexpr double pressure_at_stc = 160.472;
		constexpr double tire_load = 1345.3;
		constexpr double vehicle_speed = 35.5787;
		constexpr double expected = -8.31307e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 125") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.84946,
			.beta = -8.09755,
			.a = -1.63833,
			.b = -4.52067e-06,
			.c = 0.93252,
		};
		constexpr double pressure_at_stc = 160.261;
		constexpr double tire_load = 9512.21;
		constexpr double vehicle_speed = 99.0081;
		constexpr double expected = 1.38621e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 126") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.245702,
			.beta = -0.913603,
			.a = 9.74504,
			.b = -3.83745e-06,
			.c = 0.214241,
		};
		constexpr double pressure_at_stc = 192.519;
		constexpr double tire_load = 2721.23;
		constexpr double vehicle_speed = 1.50651;
		constexpr double expected = 0.0425276;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 127") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.03637,
			.beta = -6.15825,
			.a = 8.38379,
			.b = 3.72888e-06,
			.c = -0.255143,
		};
		constexpr double pressure_at_stc = 116.755;
		constexpr double tire_load = 7185.73;
		constexpr double vehicle_speed = 9.41735;
		constexpr double expected = -9.60648e-16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 128") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.91035,
			.beta = -5.86184,
			.a = 9.08619,
			.b = 1.90996e-07,
			.c = 0.497245,
		};
		constexpr double pressure_at_stc = 106.771;
		constexpr double tire_load = 2676.42;
		constexpr double vehicle_speed = 63.6881;
		constexpr double expected = 1.9324e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 129") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.58963,
			.beta = 5.80027,
			.a = -1.60754,
			.b = -5.26771e-06,
			.c = -0.149989,
		};
		constexpr double pressure_at_stc = 184.01;
		constexpr double tire_load = 4361.62;
		constexpr double vehicle_speed = 23.1737;
		constexpr double expected = -5.37663e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 130") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.171,
			.beta = -2.57771,
			.a = -7.76138,
			.b = 2.41766e-06,
			.c = 0.0133916,
		};
		constexpr double pressure_at_stc = 152.562;
		constexpr double tire_load = 5989.14;
		constexpr double vehicle_speed = 62.7614;
		constexpr double expected = 24183.2;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 131") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.47372,
			.beta = -7.55976,
			.a = 1.47018,
			.b = -9.24571e-06,
			.c = 0.172464,
		};
		constexpr double pressure_at_stc = 183.022;
		constexpr double tire_load = 5561.93;
		constexpr double vehicle_speed = 88.556;
		constexpr double expected = 3.1365e-46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 132") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.072272,
			.beta = -4.10373,
			.a = -4.00747,
			.b = 1.26525e-06,
			.c = -0.138793,
		};
		constexpr double pressure_at_stc = 105.642;
		constexpr double tire_load = 7774.03;
		constexpr double vehicle_speed = 71.1885;
		constexpr double expected = -1.38062e-12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 133") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.63617,
			.beta = 5.59491,
			.a = -1.69625,
			.b = -1.27245e-06,
			.c = 0.0294799,
		};
		constexpr double pressure_at_stc = 122.885;
		constexpr double tire_load = 7324.18;
		constexpr double vehicle_speed = 53.1512;
		constexpr double expected = 1.18637e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 134") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.77796,
			.beta = 6.98005,
			.a = 6.47339,
			.b = 3.93111e-06,
			.c = -0.0228985,
		};
		constexpr double pressure_at_stc = 133.448;
		constexpr double tire_load = 5786.44;
		constexpr double vehicle_speed = 85.1549;
		constexpr double expected = -1.77521e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 135") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.752965,
			.beta = 0.281875,
			.a = 6.37749,
			.b = -6.5613e-06,
			.c = -0.451598,
		};
		constexpr double pressure_at_stc = 116.426;
		constexpr double tire_load = 2008.12;
		constexpr double vehicle_speed = 49.9871;
		constexpr double expected = -4.48251e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 136") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.82754,
			.beta = -0.828731,
			.a = 8.06174,
			.b = -8.83965e-06,
			.c = 0.7004,
		};
		constexpr double pressure_at_stc = 163.667;
		constexpr double tire_load = 9836.49;
		constexpr double vehicle_speed = 31.8797;
		constexpr double expected = 2.20977e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 137") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.44922,
			.beta = -9.49757,
			.a = 2.18698,
			.b = 6.64764e-06,
			.c = 0.29569,
		};
		constexpr double pressure_at_stc = 130;
		constexpr double tire_load = 2699.77;
		constexpr double vehicle_speed = 88.6224;
		constexpr double expected = 1.06687e-46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 138") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.81035,
			.beta = -8.51347,
			.a = 2.57354,
			.b = -7.59134e-06,
			.c = -0.718015,
		};
		constexpr double pressure_at_stc = 170.055;
		constexpr double tire_load = 1797.34;
		constexpr double vehicle_speed = 37.2667;
		constexpr double expected = -4.69729e-18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 139") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.48553,
			.beta = -5.94757,
			.a = 6.55251,
			.b = -1.16181e-06,
			.c = -0.631456,
		};
		constexpr double pressure_at_stc = 179.205;
		constexpr double tire_load = 2644.36;
		constexpr double vehicle_speed = 73.4813;
		constexpr double expected = -4.74858e-31;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 140") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.11742,
			.beta = 8.10911,
			.a = -5.8585,
			.b = 3.94281e-06,
			.c = 0.465234,
		};
		constexpr double pressure_at_stc = 158.104;
		constexpr double tire_load = 3120.7;
		constexpr double vehicle_speed = 52.5831;
		constexpr double expected = 1.61392e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 141") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.11176,
			.beta = -9.26547,
			.a = -9.18593,
			.b = -8.57289e-07,
			.c = 0.24186,
		};
		constexpr double pressure_at_stc = 169.506;
		constexpr double tire_load = 2860.31;
		constexpr double vehicle_speed = 75.1584;
		constexpr double expected = 4.14657e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 142") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.02196,
			.beta = 6.20191,
			.a = 9.13404,
			.b = 2.83219e-06,
			.c = 0.72777,
		};
		constexpr double pressure_at_stc = 163.417;
		constexpr double tire_load = 6881.72;
		constexpr double vehicle_speed = 46.2124;
		constexpr double expected = 5.98383e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 143") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.7038,
			.beta = -8.85526,
			.a = 7.13133,
			.b = -1.72106e-06,
			.c = -0.981922,
		};
		constexpr double pressure_at_stc = 128.231;
		constexpr double tire_load = 445.783;
		constexpr double vehicle_speed = 28.2562;
		constexpr double expected = -2.60827e-34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 144") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.024,
			.beta = 1.09934,
			.a = -9.19221,
			.b = 1.62071e-06,
			.c = -0.709185,
		};
		constexpr double pressure_at_stc = 153.616;
		constexpr double tire_load = 9049.81;
		constexpr double vehicle_speed = 15.0455;
		constexpr double expected = -1.33603e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 145") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.34362,
			.beta = 1.08732,
			.a = -7.54282,
			.b = 5.09661e-06,
			.c = -0.179384,
		};
		constexpr double pressure_at_stc = 158.104;
		constexpr double tire_load = 201.495;
		constexpr double vehicle_speed = 21.3524;
		constexpr double expected = -4.86774e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 146") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.93756,
			.beta = -8.95134,
			.a = -1.15676,
			.b = -2.90654e-06,
			.c = 0.0111808,
		};
		constexpr double pressure_at_stc = 136.945;
		constexpr double tire_load = 2283.03;
		constexpr double vehicle_speed = 54.8212;
		constexpr double expected = 4.12745e-45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 147") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.16194,
			.beta = 9.9345,
			.a = 7.97646,
			.b = -9.27119e-06,
			.c = 0.549613,
		};
		constexpr double pressure_at_stc = 119.55;
		constexpr double tire_load = 231.545;
		constexpr double vehicle_speed = 79.6918;
		constexpr double expected = 5.2014e+34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 148") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.1136,
			.beta = 4.52788,
			.a = -9.19477,
			.b = -5.56564e-06,
			.c = -0.553594,
		};
		constexpr double pressure_at_stc = 128.369;
		constexpr double tire_load = 53.6401;
		constexpr double vehicle_speed = 98.5471;
		constexpr double expected = -1.28578e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 149") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.58729,
			.beta = -3.69839,
			.a = 5.11883,
			.b = -8.65898e-06,
			.c = 0.103944,
		};
		constexpr double pressure_at_stc = 130.947;
		constexpr double tire_load = 4492.07;
		constexpr double vehicle_speed = 52.6296;
		constexpr double expected = 1.71979e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 150") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.02197,
			.beta = -9.44699,
			.a = 9.61462,
			.b = 2.54696e-06,
			.c = -0.412581,
		};
		constexpr double pressure_at_stc = 168.053;
		constexpr double tire_load = 2960.99;
		constexpr double vehicle_speed = 73.9773;
		constexpr double expected = -1.18335e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 151") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.00737,
			.beta = -6.06271,
			.a = 3.58997,
			.b = -5.32527e-06,
			.c = 0.896431,
		};
		constexpr double pressure_at_stc = 107.677;
		constexpr double tire_load = 6525.58;
		constexpr double vehicle_speed = 80.6688;
		constexpr double expected = 8.4571e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 152") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.02375,
			.beta = -7.84452,
			.a = -1.38514,
			.b = 5.37174e-06,
			.c = 0.482243,
		};
		constexpr double pressure_at_stc = 166.573;
		constexpr double tire_load = 6924.85;
		constexpr double vehicle_speed = 49.9318;
		constexpr double expected = 1.68734e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 153") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.54698,
			.beta = -2.63601,
			.a = 3.95933,
			.b = 9.0431e-06,
			.c = 0.48233,
		};
		constexpr double pressure_at_stc = 179.259;
		constexpr double tire_load = 3196.13;
		constexpr double vehicle_speed = 34.2145;
		constexpr double expected = 7.70579e-12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 154") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.28945,
			.beta = 7.30965,
			.a = -0.427216,
			.b = 7.20938e-06,
			.c = 0.760601,
		};
		constexpr double pressure_at_stc = 102.554;
		constexpr double tire_load = 3583.26;
		constexpr double vehicle_speed = 94.5679;
		constexpr double expected = 3.84175e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 155") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.19773,
			.beta = 2.08906,
			.a = -3.81938,
			.b = 7.31243e-06,
			.c = -0.587996,
		};
		constexpr double pressure_at_stc = 179.377;
		constexpr double tire_load = 7667.14;
		constexpr double vehicle_speed = 44.0869;
		constexpr double expected = -2.15165e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 156") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.0098,
			.beta = -3.7592,
			.a = -4.26771,
			.b = 3.70186e-06,
			.c = 0.658352,
		};
		constexpr double pressure_at_stc = 125.204;
		constexpr double tire_load = 7535.66;
		constexpr double vehicle_speed = 86.5568;
		constexpr double expected = 1.03503e-14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 157") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.24444,
			.beta = -1.51893,
			.a = -7.47769,
			.b = -4.41823e-06,
			.c = -0.429404,
		};
		constexpr double pressure_at_stc = 138.209;
		constexpr double tire_load = 3917.59;
		constexpr double vehicle_speed = 90.4467;
		constexpr double expected = -10118;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 158") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.1566,
			.beta = 4.54643,
			.a = 6.66935,
			.b = -3.53339e-06,
			.c = 0.536558,
		};
		constexpr double pressure_at_stc = 118.682;
		constexpr double tire_load = 9773.44;
		constexpr double vehicle_speed = 2.56305;
		constexpr double expected = 3.59883e+30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 159") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.79635,
			.beta = -6.54221,
			.a = 7.35042,
			.b = 1.23171e-06,
			.c = -0.0968633,
		};
		constexpr double pressure_at_stc = 189.311;
		constexpr double tire_load = 3688.93;
		constexpr double vehicle_speed = 91.3179;
		constexpr double expected = -3.05389e-33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 160") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.691728,
			.beta = 5.71211,
			.a = -4.37721,
			.b = 7.82701e-06,
			.c = -0.457091,
		};
		constexpr double pressure_at_stc = 140.322;
		constexpr double tire_load = 3788.9;
		constexpr double vehicle_speed = 38.991;
		constexpr double expected = -8.13525e+22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 161") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.0228547,
			.beta = 1.46614,
			.a = 9.66271,
			.b = -1.20753e-06,
			.c = -0.262741,
		};
		constexpr double pressure_at_stc = 174.942;
		constexpr double tire_load = 8730.08;
		constexpr double vehicle_speed = 74.8438;
		constexpr double expected = -1.01638e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 162") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.29115,
			.beta = 2.12373,
			.a = -2.31442,
			.b = -1.36494e-06,
			.c = 0.128647,
		};
		constexpr double pressure_at_stc = 141.445;
		constexpr double tire_load = 8498.16;
		constexpr double vehicle_speed = 66.0233;
		constexpr double expected = 1.53998e+32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 163") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.59827,
			.beta = -6.04913,
			.a = -3.00948,
			.b = 1.73696e-07,
			.c = -0.389476,
		};
		constexpr double pressure_at_stc = 116.725;
		constexpr double tire_load = 6657.86;
		constexpr double vehicle_speed = 88.4628;
		constexpr double expected = -4.24219e-39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 164") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.90281,
			.beta = 8.33031,
			.a = 7.04474,
			.b = 4.68565e-06,
			.c = -0.594677,
		};
		constexpr double pressure_at_stc = 161.193;
		constexpr double tire_load = 9539.95;
		constexpr double vehicle_speed = 85.4032;
		constexpr double expected = -2.86024e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 165") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.52733,
			.beta = 7.19165,
			.a = 5.72273,
			.b = -8.1244e-06,
			.c = -0.332023,
		};
		constexpr double pressure_at_stc = 136.291;
		constexpr double tire_load = 3209.51;
		constexpr double vehicle_speed = 67.6841;
		constexpr double expected = -5.17373e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 166") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.77027,
			.beta = 9.36198,
			.a = 1.355,
			.b = 5.24101e-06,
			.c = -0.971752,
		};
		constexpr double pressure_at_stc = 198.836;
		constexpr double tire_load = 135.281;
		constexpr double vehicle_speed = 29.1057;
		constexpr double expected = -1.75221e+37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 167") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.27351,
			.beta = 7.1849,
			.a = 0.057662,
			.b = 7.46797e-06,
			.c = -0.859894,
		};
		constexpr double pressure_at_stc = 138.167;
		constexpr double tire_load = 4458.3;
		constexpr double vehicle_speed = 37.0006;
		constexpr double expected = -4.89606e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 168") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.35697,
			.beta = -0.0474046,
			.a = 6.29112,
			.b = -4.14318e-06,
			.c = -0.0475957,
		};
		constexpr double pressure_at_stc = 166.405;
		constexpr double tire_load = 5336.83;
		constexpr double vehicle_speed = 95.5032;
		constexpr double expected = -7.86071e-07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 169") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.42947,
			.beta = -1.19788,
			.a = -9.51062,
			.b = -5.22006e-06,
			.c = -0.220123,
		};
		constexpr double pressure_at_stc = 128.907;
		constexpr double tire_load = 6733.49;
		constexpr double vehicle_speed = 66.6858;
		constexpr double expected = -4.16006e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 170") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.3785,
			.beta = 8.2725,
			.a = -2.20885,
			.b = 6.50719e-06,
			.c = 0.487416,
		};
		constexpr double pressure_at_stc = 175.068;
		constexpr double tire_load = 6996.73;
		constexpr double vehicle_speed = 84.076;
		constexpr double expected = 7.55143e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 171") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.30035,
			.beta = -4.96204,
			.a = 2.1814,
			.b = 2.64259e-06,
			.c = -0.620598,
		};
		constexpr double pressure_at_stc = 123.336;
		constexpr double tire_load = 6098.43;
		constexpr double vehicle_speed = 35.1179;
		constexpr double expected = -1.05735e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 172") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.81547,
			.beta = 5.61078,
			.a = -0.437707,
			.b = -4.16808e-06,
			.c = -0.530746,
		};
		constexpr double pressure_at_stc = 121.87;
		constexpr double tire_load = 9740.94;
		constexpr double vehicle_speed = 23.2183;
		constexpr double expected = -8.07302e+33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 173") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.30417,
			.beta = 7.37388,
			.a = -8.37836,
			.b = -9.64811e-06,
			.c = -0.304902,
		};
		constexpr double pressure_at_stc = 156.958;
		constexpr double tire_load = 6948.12;
		constexpr double vehicle_speed = 74.5404;
		constexpr double expected = -3.42658e+35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 174") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.68772,
			.beta = 1.32754,
			.a = 4.48818,
			.b = 8.39784e-06,
			.c = 0.418214,
		};
		constexpr double pressure_at_stc = 168.737;
		constexpr double tire_load = 5175.74;
		constexpr double vehicle_speed = 23.8223;
		constexpr double expected = 270.982;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 175") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.26463,
			.beta = 9.76871,
			.a = 3.89048,
			.b = 8.48123e-06,
			.c = -0.899826,
		};
		constexpr double pressure_at_stc = 168.883;
		constexpr double tire_load = 131.799;
		constexpr double vehicle_speed = 32.7383;
		constexpr double expected = -1.64453e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 176") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.78582,
			.beta = -1.43066,
			.a = -1.11019,
			.b = 3.10684e-06,
			.c = 0.325277,
		};
		constexpr double pressure_at_stc = 169.62;
		constexpr double tire_load = 6231.83;
		constexpr double vehicle_speed = 20.9098;
		constexpr double expected = 11153.7;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 177") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.113343,
			.beta = -1.90044,
			.a = -0.52662,
			.b = 8.02953e-06,
			.c = 0.292639,
		};
		constexpr double pressure_at_stc = 154.26;
		constexpr double tire_load = 6622.53;
		constexpr double vehicle_speed = 67.4163;
		constexpr double expected = 0.000533115;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 178") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.37651,
			.beta = 6.53923,
			.a = -5.44042,
			.b = 6.27572e-06,
			.c = -0.111981,
		};
		constexpr double pressure_at_stc = 123.567;
		constexpr double tire_load = 2450.63;
		constexpr double vehicle_speed = 67.8807;
		constexpr double expected = -7.38577e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 179") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.54038,
			.beta = 8.53713,
			.a = 2.6556,
			.b = 5.14895e-06,
			.c = -0.663741,
		};
		constexpr double pressure_at_stc = 185.975;
		constexpr double tire_load = 9621.03;
		constexpr double vehicle_speed = 91.6245;
		constexpr double expected = -5.09313e+53;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 180") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.31234,
			.beta = -6.75922,
			.a = -9.69055,
			.b = -9.79448e-06,
			.c = -0.367055,
		};
		constexpr double pressure_at_stc = 171.928;
		constexpr double tire_load = 9755.55;
		constexpr double vehicle_speed = 14.4502;
		constexpr double expected = -1.26917e-27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 181") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.11409,
			.beta = -4.08327,
			.a = -9.39801,
			.b = 9.36251e-06,
			.c = 0.18201,
		};
		constexpr double pressure_at_stc = 120.297;
		constexpr double tire_load = 4221.93;
		constexpr double vehicle_speed = 58.0979;
		constexpr double expected = 1.13821e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 182") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.09666,
			.beta = -7.51136,
			.a = 9.6556,
			.b = -9.33855e-06,
			.c = -0.205799,
		};
		constexpr double pressure_at_stc = 169.044;
		constexpr double tire_load = 2816.2;
		constexpr double vehicle_speed = 0.382441;
		constexpr double expected = 2.10158e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 183") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.02448,
			.beta = -4.38422,
			.a = 5.45999,
			.b = -6.22399e-06,
			.c = 0.689035,
		};
		constexpr double pressure_at_stc = 119.76;
		constexpr double tire_load = 8912.11;
		constexpr double vehicle_speed = 5.04499;
		constexpr double expected = 6382.12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 184") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.41193,
			.beta = -7.16707,
			.a = 4.27129,
			.b = 8.9821e-06,
			.c = 0.832388,
		};
		constexpr double pressure_at_stc = 166.097;
		constexpr double tire_load = 9524.57;
		constexpr double vehicle_speed = 27.338;
		constexpr double expected = 4.23477e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 185") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.50691,
			.beta = 1.67343,
			.a = 5.16579,
			.b = 9.96474e-06,
			.c = 0.670499,
		};
		constexpr double pressure_at_stc = 100.422;
		constexpr double tire_load = 3663.21;
		constexpr double vehicle_speed = 21.8064;
		constexpr double expected = 0.0360328;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 186") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.44927,
			.beta = 4.47492,
			.a = 7.71623,
			.b = -3.98277e-06,
			.c = -0.388805,
		};
		constexpr double pressure_at_stc = 146.05;
		constexpr double tire_load = 521.155;
		constexpr double vehicle_speed = 40.4856;
		constexpr double expected = -2.78167e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 187") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.17697,
			.beta = -0.253169,
			.a = 7.16996,
			.b = 5.07902e-06,
			.c = -0.375954,
		};
		constexpr double pressure_at_stc = 155.525;
		constexpr double tire_load = 8559.88;
		constexpr double vehicle_speed = 69.9455;
		constexpr double expected = -4.47778e-13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 188") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.63946,
			.beta = -1.7542,
			.a = 9.86708,
			.b = 7.69219e-06,
			.c = -0.867838,
		};
		constexpr double pressure_at_stc = 140.52;
		constexpr double tire_load = 7829.63;
		constexpr double vehicle_speed = 71.0241;
		constexpr double expected = -4.60853e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 189") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.29489,
			.beta = -1.02521,
			.a = -6.9195,
			.b = -9.82943e-07,
			.c = 0.347891,
		};
		constexpr double pressure_at_stc = 158.559;
		constexpr double tire_load = 7479.54;
		constexpr double vehicle_speed = 13.6549;
		constexpr double expected = 0.000126043;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 190") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.86805,
			.beta = 0.0957221,
			.a = 2.94036,
			.b = 2.77406e-06,
			.c = 0.405758,
		};
		constexpr double pressure_at_stc = 143.907;
		constexpr double tire_load = 3283.3;
		constexpr double vehicle_speed = 63.5488;
		constexpr double expected = 7.1322e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 191") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.92659,
			.beta = 6.91062,
			.a = 6.90995,
			.b = -2.6215e-06,
			.c = -0.934834,
		};
		constexpr double pressure_at_stc = 185.213;
		constexpr double tire_load = 3531.31;
		constexpr double vehicle_speed = 53.7106;
		constexpr double expected = -3.73284e+51;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 192") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.59465,
			.beta = -7.50058,
			.a = 5.03083,
			.b = -6.83206e-06,
			.c = 0.97889,
		};
		constexpr double pressure_at_stc = 168.284;
		constexpr double tire_load = 3174.64;
		constexpr double vehicle_speed = 5.15885;
		constexpr double expected = 1.85485e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 193") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.24317,
			.beta = 9.77938,
			.a = -0.557265,
			.b = 2.29726e-07,
			.c = 0.442506,
		};
		constexpr double pressure_at_stc = 158.424;
		constexpr double tire_load = 3782.04;
		constexpr double vehicle_speed = 22.2759;
		constexpr double expected = 5.09675e+35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 194") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.89899,
			.beta = 7.64061,
			.a = 7.1197,
			.b = 5.49935e-06,
			.c = -0.0148019,
		};
		constexpr double pressure_at_stc = 195.109;
		constexpr double tire_load = 6722.22;
		constexpr double vehicle_speed = 23.7401;
		constexpr double expected = -1.50872e+40;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 195") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.14246,
			.beta = -9.67457,
			.a = 5.71368,
			.b = -1.73053e-06,
			.c = 0.702591,
		};
		constexpr double pressure_at_stc = 102.856;
		constexpr double tire_load = 8660.98;
		constexpr double vehicle_speed = 52.6782;
		constexpr double expected = 9.39337e-43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 196") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.86956,
			.beta = 2.92971,
			.a = 0.298517,
			.b = 4.20265e-06,
			.c = -0.445022,
		};
		constexpr double pressure_at_stc = 134.319;
		constexpr double tire_load = 1470.57;
		constexpr double vehicle_speed = 5.87916;
		constexpr double expected = -3.82431e+32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 197") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.17247,
			.beta = -7.90645,
			.a = -6.13909,
			.b = 6.22764e-06,
			.c = 0.648245,
		};
		constexpr double pressure_at_stc = 128.852;
		constexpr double tire_load = 188.453;
		constexpr double vehicle_speed = 27.3263;
		constexpr double expected = 1129.71;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 198") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.89436,
			.beta = -1.11628,
			.a = 3.15706,
			.b = 6.36394e-07,
			.c = -0.695918,
		};
		constexpr double pressure_at_stc = 151.813;
		constexpr double tire_load = 8107.25;
		constexpr double vehicle_speed = 40.3522;
		constexpr double expected = -1.98713e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 199") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.818731,
			.beta = -7.78388,
			.a = 7.95563,
			.b = 2.10343e-06,
			.c = 0.861711,
		};
		constexpr double pressure_at_stc = 133.548;
		constexpr double tire_load = 6650.12;
		constexpr double vehicle_speed = 55.9987;
		constexpr double expected = 3.37517e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 200") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.27368,
			.beta = 5.27218,
			.a = -0.110442,
			.b = 8.78617e-06,
			.c = 0.308209,
		};
		constexpr double pressure_at_stc = 102.706;
		constexpr double tire_load = 1849.42;
		constexpr double vehicle_speed = 61.8886;
		constexpr double expected = 6.31776e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 201") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.61381,
			.beta = 6.58602,
			.a = 5.44836,
			.b = 6.74391e-06,
			.c = 0.104778,
		};
		constexpr double pressure_at_stc = 122.273;
		constexpr double tire_load = 8347.34;
		constexpr double vehicle_speed = 17.7813;
		constexpr double expected = 2.78978e+46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 202") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.71877,
			.beta = 9.04888,
			.a = -3.84486,
			.b = 6.1315e-06,
			.c = -0.841916,
		};
		constexpr double pressure_at_stc = 155.102;
		constexpr double tire_load = 942.043;
		constexpr double vehicle_speed = 0.236361;
		constexpr double expected = -2.9491e+44;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 203") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.60919,
			.beta = -7.28288,
			.a = 1.68295,
			.b = -1.25492e-06,
			.c = 0.863628,
		};
		constexpr double pressure_at_stc = 144.815;
		constexpr double tire_load = 5191.71;
		constexpr double vehicle_speed = 74.9898;
		constexpr double expected = 0.000220629;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 204") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.52353,
			.beta = -0.0497708,
			.a = 4.27075,
			.b = -3.39808e-06,
			.c = 0.108476,
		};
		constexpr double pressure_at_stc = 155.935;
		constexpr double tire_load = 5781.73;
		constexpr double vehicle_speed = 24.8865;
		constexpr double expected = 1.06621e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 205") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.84663,
			.beta = -6.60162,
			.a = -5.27136,
			.b = 7.11126e-06,
			.c = 0.513414,
		};
		constexpr double pressure_at_stc = 111.992;
		constexpr double tire_load = 4536.17;
		constexpr double vehicle_speed = 43.7599;
		constexpr double expected = 8.60878e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 206") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.65775,
			.beta = -8.82295,
			.a = -6.04069,
			.b = 3.32778e-06,
			.c = -0.611743,
		};
		constexpr double pressure_at_stc = 154.568;
		constexpr double tire_load = 2873.54;
		constexpr double vehicle_speed = 16.1124;
		constexpr double expected = -5.67771e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 207") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.26736,
			.beta = 5.45779,
			.a = 4.01823,
			.b = 3.87133e-06,
			.c = 0.268143,
		};
		constexpr double pressure_at_stc = 110.38;
		constexpr double tire_load = 8911;
		constexpr double vehicle_speed = 42.7265;
		constexpr double expected = 3.97988e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 208") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.83145,
			.beta = -8.99464,
			.a = -0.406803,
			.b = 9.9295e-06,
			.c = -0.748964,
		};
		constexpr double pressure_at_stc = 145.453;
		constexpr double tire_load = 7258.01;
		constexpr double vehicle_speed = 69.848;
		constexpr double expected = -4.85272e-52;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 209") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.39094,
			.beta = 3.04123,
			.a = -7.75294,
			.b = -3.35726e-06,
			.c = 0.640896,
		};
		constexpr double pressure_at_stc = 174.091;
		constexpr double tire_load = 9044.2;
		constexpr double vehicle_speed = 75.2631;
		constexpr double expected = 2.00927e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 210") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.6239,
			.beta = 8.53508,
			.a = 6.47215,
			.b = 4.43633e-06,
			.c = -0.223271,
		};
		constexpr double pressure_at_stc = 139.649;
		constexpr double tire_load = 2852.04;
		constexpr double vehicle_speed = 9.56935;
		constexpr double expected = -9.2463e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 211") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.23331,
			.beta = 5.60469,
			.a = 4.14087,
			.b = -1.65462e-06,
			.c = -0.339864,
		};
		constexpr double pressure_at_stc = 155.086;
		constexpr double tire_load = 5276.39;
		constexpr double vehicle_speed = 56.6772;
		constexpr double expected = -1.73533e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 212") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.61706,
			.beta = 7.73509,
			.a = 9.21224,
			.b = -1.83646e-06,
			.c = -0.654298,
		};
		constexpr double pressure_at_stc = 142.96;
		constexpr double tire_load = 3176.68;
		constexpr double vehicle_speed = 67.1008;
		constexpr double expected = -1.07025e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 213") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.33277,
			.beta = 7.25838,
			.a = 7.28369,
			.b = -9.82486e-06,
			.c = -0.290956,
		};
		constexpr double pressure_at_stc = 130.905;
		constexpr double tire_load = 6750.61;
		constexpr double vehicle_speed = 58.5153;
		constexpr double expected = -1.56603e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 214") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.29622,
			.beta = 3.62295,
			.a = 5.74621,
			.b = 5.59721e-06,
			.c = 0.740227,
		};
		constexpr double pressure_at_stc = 177.77;
		constexpr double tire_load = 5296.57;
		constexpr double vehicle_speed = 99.5037;
		constexpr double expected = 3.57896e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 215") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.94471,
			.beta = -6.02137,
			.a = 7.6902,
			.b = 9.18918e-06,
			.c = -0.885561,
		};
		constexpr double pressure_at_stc = 101.054;
		constexpr double tire_load = 9530.8;
		constexpr double vehicle_speed = 74.9817;
		constexpr double expected = -8.57792e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 216") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.95797,
			.beta = 4.27183,
			.a = 1.82727,
			.b = -8.91811e-06,
			.c = -0.34837,
		};
		constexpr double pressure_at_stc = 107.145;
		constexpr double tire_load = 4495.72;
		constexpr double vehicle_speed = 48.3466;
		constexpr double expected = -4.19617e+13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 217") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.02505,
			.beta = -9.82344,
			.a = -0.332487,
			.b = 6.33904e-06,
			.c = 0.814215,
		};
		constexpr double pressure_at_stc = 160.875;
		constexpr double tire_load = 3817.33;
		constexpr double vehicle_speed = 10.2155;
		constexpr double expected = 1.31303e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 218") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.655438,
			.beta = -5.66669,
			.a = 0.187529,
			.b = 4.62737e-06,
			.c = -0.190566,
		};
		constexpr double pressure_at_stc = 193.206;
		constexpr double tire_load = 6220.08;
		constexpr double vehicle_speed = 42.4669;
		constexpr double expected = -4.4545e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 219") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.30989,
			.beta = 6.18879,
			.a = -2.02255,
			.b = 9.05089e-06,
			.c = 0.444703,
		};
		constexpr double pressure_at_stc = 116.16;
		constexpr double tire_load = 5579.63;
		constexpr double vehicle_speed = 33.9322;
		constexpr double expected = 1.2707e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 220") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.9865,
			.beta = 8.67179,
			.a = 9.07116,
			.b = -9.67228e-06,
			.c = -0.569547,
		};
		constexpr double pressure_at_stc = 110.746;
		constexpr double tire_load = 2004.21;
		constexpr double vehicle_speed = 80.8857;
		constexpr double expected = -3.5967e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 221") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.56428,
			.beta = 0.488458,
			.a = 3.06565,
			.b = -1.82198e-06,
			.c = 0.191673,
		};
		constexpr double pressure_at_stc = 131.789;
		constexpr double tire_load = 9652.02;
		constexpr double vehicle_speed = 32.5071;
		constexpr double expected = 1.45057e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 222") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.17171,
			.beta = -0.504743,
			.a = -4.45523,
			.b = 2.63667e-07,
			.c = -0.523836,
		};
		constexpr double pressure_at_stc = 132.236;
		constexpr double tire_load = 4137.72;
		constexpr double vehicle_speed = 58.4116;
		constexpr double expected = -5.66321e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 223") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.24082,
			.beta = 9.85955,
			.a = -3.34994,
			.b = 1.19332e-06,
			.c = 0.596477,
		};
		constexpr double pressure_at_stc = 112.423;
		constexpr double tire_load = 5682.75;
		constexpr double vehicle_speed = 38.8052;
		constexpr double expected = 4.78311e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 224") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.50122,
			.beta = -6.91626,
			.a = 0.486248,
			.b = 5.56766e-06,
			.c = 0.996784,
		};
		constexpr double pressure_at_stc = 166.831;
		constexpr double tire_load = 9944.5;
		constexpr double vehicle_speed = 40.6159;
		constexpr double expected = 6.1408e-43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 225") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.43595,
			.beta = 4.3244,
			.a = 5.00504,
			.b = 8.86884e-06,
			.c = 0.0284948,
		};
		constexpr double pressure_at_stc = 134.809;
		constexpr double tire_load = 6489.88;
		constexpr double vehicle_speed = 8.6866;
		constexpr double expected = 5.11926e+31;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 226") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.33336,
			.beta = 2.41244,
			.a = 1.25269,
			.b = -2.16389e-06,
			.c = 0.464214,
		};
		constexpr double pressure_at_stc = 177.964;
		constexpr double tire_load = 2293.43;
		constexpr double vehicle_speed = 23.0058;
		constexpr double expected = 4.04565e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 227") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.5727,
			.beta = -5.48575,
			.a = -1.48735,
			.b = 8.8292e-06,
			.c = 0.55792,
		};
		constexpr double pressure_at_stc = 178.04;
		constexpr double tire_load = 7354.25;
		constexpr double vehicle_speed = 33.1491;
		constexpr double expected = 4.4328e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 228") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.06016,
			.beta = -6.42401,
			.a = 1.11532,
			.b = -6.80822e-06,
			.c = 0.732056,
		};
		constexpr double pressure_at_stc = 124.345;
		constexpr double tire_load = 7091.13;
		constexpr double vehicle_speed = 27.4312;
		constexpr double expected = 5.1976e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 229") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.31134,
			.beta = -1.65959,
			.a = 7.51246,
			.b = 8.90951e-06,
			.c = -0.218566,
		};
		constexpr double pressure_at_stc = 147.604;
		constexpr double tire_load = 4579.59;
		constexpr double vehicle_speed = 58.085;
		constexpr double expected = -5.27109e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 230") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.39564,
			.beta = -9.64731,
			.a = -6.46126,
			.b = 7.86112e-06,
			.c = 0.497687,
		};
		constexpr double pressure_at_stc = 182.053;
		constexpr double tire_load = 547.417;
		constexpr double vehicle_speed = 89.587;
		constexpr double expected = 2.29912e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 231") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.3264,
			.beta = -2.45149,
			.a = 4.49741,
			.b = 9.92407e-06,
			.c = 0.916176,
		};
		constexpr double pressure_at_stc = 151.661;
		constexpr double tire_load = 8318.23;
		constexpr double vehicle_speed = 97.5202;
		constexpr double expected = 2.91687e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 232") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.938812,
			.beta = 4.8538,
			.a = 6.23928,
			.b = 9.2086e-07,
			.c = 0.594534,
		};
		constexpr double pressure_at_stc = 194.399;
		constexpr double tire_load = 702.707;
		constexpr double vehicle_speed = 96.1175;
		constexpr double expected = 3.32235e+16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 233") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.79014,
			.beta = 2.7959,
			.a = -9.78402,
			.b = -9.58159e-06,
			.c = -0.0443029,
		};
		constexpr double pressure_at_stc = 193.788;
		constexpr double tire_load = 2741.05;
		constexpr double vehicle_speed = 14.6618;
		constexpr double expected = -2.20469e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 234") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.33194,
			.beta = -3.06746,
			.a = -0.00678472,
			.b = -6.04363e-06,
			.c = -0.933964,
		};
		constexpr double pressure_at_stc = 115.204;
		constexpr double tire_load = 2551.06;
		constexpr double vehicle_speed = 75.5293;
		constexpr double expected = -0.157187;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 235") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.84258,
			.beta = -9.88539,
			.a = -0.907981,
			.b = -1.1042e-07,
			.c = 0.162268,
		};
		constexpr double pressure_at_stc = 136.303;
		constexpr double tire_load = 3890.37;
		constexpr double vehicle_speed = 32.0666;
		constexpr double expected = 2.83033e-18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 236") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.41165,
			.beta = -7.45377,
			.a = 8.344,
			.b = 4.25196e-06,
			.c = -0.688567,
		};
		constexpr double pressure_at_stc = 120.088;
		constexpr double tire_load = 2259.32;
		constexpr double vehicle_speed = 71.0276;
		constexpr double expected = -3.62281e-28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 237") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.6775,
			.beta = 3.235,
			.a = 1.56512,
			.b = -7.95076e-06,
			.c = 0.0605155,
		};
		constexpr double pressure_at_stc = 183.72;
		constexpr double tire_load = 4781.33;
		constexpr double vehicle_speed = 42.9399;
		constexpr double expected = 9.44883e+36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 238") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.243997,
			.beta = -9.18256,
			.a = -1.76362,
			.b = -8.37195e-06,
			.c = -0.687597,
		};
		constexpr double pressure_at_stc = 118.639;
		constexpr double tire_load = 9641.02;
		constexpr double vehicle_speed = 94.9164;
		constexpr double expected = -6.70332e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 239") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.157935,
			.beta = 7.81502,
			.a = 7.59764,
			.b = -3.13341e-06,
			.c = -0.693885,
		};
		constexpr double pressure_at_stc = 175.452;
		constexpr double tire_load = 538.951;
		constexpr double vehicle_speed = 43.1592;
		constexpr double expected = -8.42118e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 240") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.63787,
			.beta = -5.51865,
			.a = -2.54394,
			.b = 9.51405e-06,
			.c = 0.486444,
		};
		constexpr double pressure_at_stc = 138.382;
		constexpr double tire_load = 7266;
		constexpr double vehicle_speed = 89.7069;
		constexpr double expected = 7.99703e-14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 241") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.12255,
			.beta = 5.65535,
			.a = 8.66628,
			.b = -1.84853e-06,
			.c = -0.481275,
		};
		constexpr double pressure_at_stc = 187.831;
		constexpr double tire_load = 3174.29;
		constexpr double vehicle_speed = 4.75261;
		constexpr double expected = -1.25356e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 242") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.0429,
			.beta = -0.333207,
			.a = 8.56932,
			.b = -3.24321e-06,
			.c = 0.0354744,
		};
		constexpr double pressure_at_stc = 195.916;
		constexpr double tire_load = 2143.7;
		constexpr double vehicle_speed = 81.3914;
		constexpr double expected = 6.454e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 243") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.773006,
			.beta = 1.3153,
			.a = 0.385445,
			.b = 8.24972e-06,
			.c = 0.384808,
		};
		constexpr double pressure_at_stc = 100.549;
		constexpr double tire_load = 1220.39;
		constexpr double vehicle_speed = 42.4868;
		constexpr double expected = 2.92553e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 244") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.48591,
			.beta = -2.49161,
			.a = 4.771,
			.b = 2.21073e-06,
			.c = 0.802396,
		};
		constexpr double pressure_at_stc = 168.631;
		constexpr double tire_load = 1279.75;
		constexpr double vehicle_speed = 12.1003;
		constexpr double expected = 1.29678e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 245") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.7299,
			.beta = -3.4013,
			.a = -2.75223,
			.b = -6.67992e-06,
			.c = -0.073324,
		};
		constexpr double pressure_at_stc = 141.036;
		constexpr double tire_load = 9900.69;
		constexpr double vehicle_speed = 55.6972;
		constexpr double expected = -1.30623e-29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 246") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.8407,
			.beta = 6.853,
			.a = -5.85969,
			.b = -2.687e-06,
			.c = -0.320894,
		};
		constexpr double pressure_at_stc = 184.153;
		constexpr double tire_load = 2930.33;
		constexpr double vehicle_speed = 33.3838;
		constexpr double expected = -4.62133e+09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 247") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.79237,
			.beta = -7.80573,
			.a = 2.05087,
			.b = 3.56731e-06,
			.c = 0.49676,
		};
		constexpr double pressure_at_stc = 150.9;
		constexpr double tire_load = 4783.75;
		constexpr double vehicle_speed = 45.6038;
		constexpr double expected = 3.15165e-29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 248") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.81531,
			.beta = -4.21438,
			.a = 3.99095,
			.b = -5.93353e-06,
			.c = 0.659581,
		};
		constexpr double pressure_at_stc = 196.964;
		constexpr double tire_load = 892.936;
		constexpr double vehicle_speed = 42.1069;
		constexpr double expected = 3.15215;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 249") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.03908,
			.beta = -4.1136,
			.a = 7.63633,
			.b = 1.0921e-06,
			.c = -0.862658,
		};
		constexpr double pressure_at_stc = 134.916;
		constexpr double tire_load = 3359.27;
		constexpr double vehicle_speed = 90.4321;
		constexpr double expected = -2084.94;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 250") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.43449,
			.beta = 9.29734,
			.a = 5.78129,
			.b = -2.36904e-06,
			.c = -0.753298,
		};
		constexpr double pressure_at_stc = 130.269;
		constexpr double tire_load = 9479.52;
		constexpr double vehicle_speed = 49.6999;
		constexpr double expected = -4.16006e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 251") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.598223,
			.beta = 7.58305,
			.a = 0.596452,
			.b = 2.13062e-06,
			.c = -0.358165,
		};
		constexpr double pressure_at_stc = 126.538;
		constexpr double tire_load = 2677.82;
		constexpr double vehicle_speed = 44.6428;
		constexpr double expected = -5.03047e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 252") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.23713,
			.beta = -9.91077,
			.a = 1.81913,
			.b = 5.37926e-06,
			.c = 0.0443792,
		};
		constexpr double pressure_at_stc = 137.032;
		constexpr double tire_load = 9185.72;
		constexpr double vehicle_speed = 57.2089;
		constexpr double expected = 3.97099e-19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 253") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.63491,
			.beta = -8.93179,
			.a = -9.36619,
			.b = -1.85906e-06,
			.c = 0.397577,
		};
		constexpr double pressure_at_stc = 175.954;
		constexpr double tire_load = 5200.64;
		constexpr double vehicle_speed = 17.4885;
		constexpr double expected = 4.53446e-18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 254") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.63812,
			.beta = 6.79536,
			.a = -9.12405,
			.b = 7.69652e-06,
			.c = -0.0965439,
		};
		constexpr double pressure_at_stc = 145.217;
		constexpr double tire_load = 3366.98;
		constexpr double vehicle_speed = 16.8933;
		constexpr double expected = -2.2022e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 255") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.29704,
			.beta = 0.0576781,
			.a = -0.828927,
			.b = 6.52319e-06,
			.c = -0.729717,
		};
		constexpr double pressure_at_stc = 133.052;
		constexpr double tire_load = 6192.5;
		constexpr double vehicle_speed = 79.7329;
		constexpr double expected = -5.55731e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 256") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.837292,
			.beta = -6.50632,
			.a = -7.132,
			.b = 3.65451e-06,
			.c = 0.657269,
		};
		constexpr double pressure_at_stc = 165.394;
		constexpr double tire_load = 1932.95;
		constexpr double vehicle_speed = 42.3193;
		constexpr double expected = 4.56647e-16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 257") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.63341,
			.beta = -2.23651,
			.a = 4.53244,
			.b = 1.98172e-06,
			.c = 0.10689,
		};
		constexpr double pressure_at_stc = 137.472;
		constexpr double tire_load = 4922.34;
		constexpr double vehicle_speed = 29.9437;
		constexpr double expected = 1.61199e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 258") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.55543,
			.beta = 4.55475,
			.a = 9.56201,
			.b = 2.13593e-07,
			.c = 0.471974,
		};
		constexpr double pressure_at_stc = 101.925;
		constexpr double tire_load = 7514.81;
		constexpr double vehicle_speed = 43.1329;
		constexpr double expected = 3.44285e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 259") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.05197,
			.beta = 3.8337,
			.a = -0.255642,
			.b = 1.21311e-06,
			.c = -0.91335,
		};
		constexpr double pressure_at_stc = 168.702;
		constexpr double tire_load = 667.168;
		constexpr double vehicle_speed = 89.6755;
		constexpr double expected = -2.904e+13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 260") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.7826,
			.beta = 1.27105,
			.a = -4.88724,
			.b = 8.75437e-06,
			.c = -0.448947,
		};
		constexpr double pressure_at_stc = 184.694;
		constexpr double tire_load = 4132.57;
		constexpr double vehicle_speed = 27.3847;
		constexpr double expected = -0.00249547;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 261") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.41942,
			.beta = -8.64394,
			.a = 5.06564,
			.b = -9.58809e-06,
			.c = -0.0665582,
		};
		constexpr double pressure_at_stc = 106.768;
		constexpr double tire_load = 5666.84;
		constexpr double vehicle_speed = 54.1463;
		constexpr double expected = -8.37414e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 262") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.199393,
			.beta = -2.63239,
			.a = -0.210397,
			.b = -4.76987e-06,
			.c = 0.571574,
		};
		constexpr double pressure_at_stc = 140.206;
		constexpr double tire_load = 4074.9;
		constexpr double vehicle_speed = 13.3662;
		constexpr double expected = 1.11288e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 263") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.49166,
			.beta = 0.983739,
			.a = 9.567,
			.b = 7.4633e-06,
			.c = -0.817467,
		};
		constexpr double pressure_at_stc = 156.641;
		constexpr double tire_load = 4136.23;
		constexpr double vehicle_speed = 61.2699;
		constexpr double expected = -1.19705e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 264") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.13943,
			.beta = -0.769423,
			.a = 9.70925,
			.b = 3.40047e-06,
			.c = 0.53173,
		};
		constexpr double pressure_at_stc = 118.723;
		constexpr double tire_load = 2940.26;
		constexpr double vehicle_speed = 81.7049;
		constexpr double expected = 2.14936e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 265") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.704597,
			.beta = -9.03752,
			.a = -2.36453,
			.b = -2.18374e-07,
			.c = 0.866619,
		};
		constexpr double pressure_at_stc = 199.253;
		constexpr double tire_load = 3904.99;
		constexpr double vehicle_speed = 87.1854;
		constexpr double expected = 7.10935e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 266") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.68028,
			.beta = 5.74992,
			.a = 7.98327,
			.b = 7.14093e-06,
			.c = 0.525991,
		};
		constexpr double pressure_at_stc = 170.213;
		constexpr double tire_load = 2243.93;
		constexpr double vehicle_speed = 25.6903;
		constexpr double expected = 1.93889e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 267") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.68577,
			.beta = -7.68409,
			.a = -9.58638,
			.b = -3.28426e-07,
			.c = 0.784339,
		};
		constexpr double pressure_at_stc = 149.744;
		constexpr double tire_load = 4032.6;
		constexpr double vehicle_speed = 59.9638;
		constexpr double expected = 1.54877e-27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 268") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.13965,
			.beta = 4.85276,
			.a = -8.2743,
			.b = 1.71052e-06,
			.c = -0.549505,
		};
		constexpr double pressure_at_stc = 157.581;
		constexpr double tire_load = 4710.58;
		constexpr double vehicle_speed = 98.7207;
		constexpr double expected = -1.4508e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 269") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.58351,
			.beta = 0.00514497,
			.a = 1.32886,
			.b = -4.37675e-06,
			.c = -0.64128,
		};
		constexpr double pressure_at_stc = 108.153;
		constexpr double tire_load = 6189.07;
		constexpr double vehicle_speed = 59.4494;
		constexpr double expected = -0.00157926;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 270") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.143281,
			.beta = 7.12775,
			.a = -1.27395,
			.b = -1.90115e-06,
			.c = 0.592204,
		};
		constexpr double pressure_at_stc = 118.087;
		constexpr double tire_load = 2223.29;
		constexpr double vehicle_speed = 7.78893;
		constexpr double expected = 1.68454e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 271") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.26041,
			.beta = 1.37227,
			.a = 8.99202,
			.b = 5.39192e-06,
			.c = -0.971014,
		};
		constexpr double pressure_at_stc = 169.462;
		constexpr double tire_load = 5061.16;
		constexpr double vehicle_speed = 15.4595;
		constexpr double expected = -0.000682818;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 272") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.61485,
			.beta = 3.18158,
			.a = 0.624621,
			.b = 1.35481e-06,
			.c = -0.282705,
		};
		constexpr double pressure_at_stc = 125.089;
		constexpr double tire_load = 3413.41;
		constexpr double vehicle_speed = 40.3207;
		constexpr double expected = -6.18932e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 273") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.85891,
			.beta = -9.04723,
			.a = -8.16808,
			.b = 1.84965e-06,
			.c = -0.685473,
		};
		constexpr double pressure_at_stc = 128.869;
		constexpr double tire_load = 5408.56;
		constexpr double vehicle_speed = 92.1151;
		constexpr double expected = -2.5682e-48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 274") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.68186,
			.beta = 1.96951,
			.a = 9.06094,
			.b = -8.90337e-06,
			.c = -0.98439,
		};
		constexpr double pressure_at_stc = 107.524;
		constexpr double tire_load = 9907.11;
		constexpr double vehicle_speed = 96.12;
		constexpr double expected = -0.232917;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 275") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.55599,
			.beta = 9.19376,
			.a = -2.60597,
			.b = -1.5691e-06,
			.c = -0.13872,
		};
		constexpr double pressure_at_stc = 133.001;
		constexpr double tire_load = 2882.02;
		constexpr double vehicle_speed = 18.2849;
		constexpr double expected = -7.81839e+37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 276") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.13713,
			.beta = -4.08795,
			.a = 8.65579,
			.b = 9.89778e-06,
			.c = 0.608441,
		};
		constexpr double pressure_at_stc = 154.22;
		constexpr double tire_load = 7915.38;
		constexpr double vehicle_speed = 55.9858;
		constexpr double expected = 2.09349e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 277") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.13623,
			.beta = -9.46787,
			.a = -1.9754,
			.b = 5.51924e-06,
			.c = -0.732925,
		};
		constexpr double pressure_at_stc = 140.855;
		constexpr double tire_load = 2104.83;
		constexpr double vehicle_speed = 81.3622;
		constexpr double expected = -5.55238e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 278") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.97039,
			.beta = -6.01493,
			.a = 1.83218,
			.b = -2.42404e-06,
			.c = 0.816299,
		};
		constexpr double pressure_at_stc = 126.243;
		constexpr double tire_load = 7071.32;
		constexpr double vehicle_speed = 22.6196;
		constexpr double expected = 1.33081e-07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 279") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.42872,
			.beta = -8.67025,
			.a = -8.84924,
			.b = 7.78711e-06,
			.c = 0.994824,
		};
		constexpr double pressure_at_stc = 196.343;
		constexpr double tire_load = 7770.01;
		constexpr double vehicle_speed = 76.5018;
		constexpr double expected = 5.19781e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 280") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.69689,
			.beta = -5.40044,
			.a = -1.54772,
			.b = -4.91659e-06,
			.c = 0.777426,
		};
		constexpr double pressure_at_stc = 184.847;
		constexpr double tire_load = 9177.09;
		constexpr double vehicle_speed = 0.221863;
		constexpr double expected = -2.7604e-37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 281") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.13176,
			.beta = 9.83709,
			.a = -8.42433,
			.b = -9.79176e-06,
			.c = 0.774751,
		};
		constexpr double pressure_at_stc = 111.466;
		constexpr double tire_load = 6995.44;
		constexpr double vehicle_speed = 48.1766;
		constexpr double expected = 3.98316e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 282") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.56511,
			.beta = 3.12826,
			.a = -5.96185,
			.b = -9.42609e-06,
			.c = 0.356588,
		};
		constexpr double pressure_at_stc = 135.448;
		constexpr double tire_load = 8362.01;
		constexpr double vehicle_speed = 57.7029;
		constexpr double expected = 289.503;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 283") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.47674,
			.beta = 6.00413,
			.a = -5.82567,
			.b = -7.77104e-07,
			.c = 0.857459,
		};
		constexpr double pressure_at_stc = 132.467;
		constexpr double tire_load = 55.8029;
		constexpr double vehicle_speed = 10.2427;
		constexpr double expected = 3.6564e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 284") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.562788,
			.beta = -1.04363,
			.a = 0.380028,
			.b = -3.3637e-06,
			.c = 0.336522,
		};
		constexpr double pressure_at_stc = 131.825;
		constexpr double tire_load = 1834.66;
		constexpr double vehicle_speed = 41.331;
		constexpr double expected = 0.187563;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 285") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.887289,
			.beta = 2.89087,
			.a = 6.10649,
			.b = 7.57127e-06,
			.c = 0.414686,
		};
		constexpr double pressure_at_stc = 143.153;
		constexpr double tire_load = 9224.27;
		constexpr double vehicle_speed = 8.87789;
		constexpr double expected = 1.52204e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 286") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.44004,
			.beta = 8.27012,
			.a = -5.08236,
			.b = 2.97236e-06,
			.c = 0.402771,
		};
		constexpr double pressure_at_stc = 169.494;
		constexpr double tire_load = 6628.77;
		constexpr double vehicle_speed = 60.6864;
		constexpr double expected = 1.1839e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 287") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.04488,
			.beta = 5.99123,
			.a = 6.47004,
			.b = 7.94112e-06,
			.c = -0.262075,
		};
		constexpr double pressure_at_stc = 126.985;
		constexpr double tire_load = 4643.22;
		constexpr double vehicle_speed = 13.0052;
		constexpr double expected = -7.98788e+09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 288") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.38957,
			.beta = 1.27636,
			.a = -1.49781,
			.b = -4.53112e-08,
			.c = -0.597937,
		};
		constexpr double pressure_at_stc = 139.879;
		constexpr double tire_load = 8572.44;
		constexpr double vehicle_speed = 47.9074;
		constexpr double expected = -1.78575e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 289") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.40936,
			.beta = 8.99444,
			.a = -0.865019,
			.b = 7.6568e-06,
			.c = 0.689663,
		};
		constexpr double pressure_at_stc = 163.515;
		constexpr double tire_load = 3257.81;
		constexpr double vehicle_speed = 70.7503;
		constexpr double expected = 2.72246e+50;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 290") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.05226,
			.beta = 0.48915,
			.a = -3.81556,
			.b = 6.82333e-06,
			.c = 0.680728,
		};
		constexpr double pressure_at_stc = 122.084;
		constexpr double tire_load = 2391.7;
		constexpr double vehicle_speed = 97.1333;
		constexpr double expected = 23838;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 291") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.83959,
			.beta = 7.72767,
			.a = -7.97749,
			.b = -6.26968e-06,
			.c = -0.989209,
		};
		constexpr double pressure_at_stc = 152.765;
		constexpr double tire_load = 2523.19;
		constexpr double vehicle_speed = 66.3949;
		constexpr double expected = -1.14574e+35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 292") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.22583,
			.beta = 2.22886,
			.a = 2.46991,
			.b = 2.25505e-06,
			.c = 0.614885,
		};
		constexpr double pressure_at_stc = 123.303;
		constexpr double tire_load = 6840.02;
		constexpr double vehicle_speed = 47.2195;
		constexpr double expected = 1.12818e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 293") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.80359,
			.beta = -2.14538,
			.a = 3.84505,
			.b = -2.26631e-06,
			.c = -0.230069,
		};
		constexpr double pressure_at_stc = 109.66;
		constexpr double tire_load = 6787.48;
		constexpr double vehicle_speed = 13.7683;
		constexpr double expected = -6.44685e-12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 294") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.80122,
			.beta = -8.42956,
			.a = 7.07803,
			.b = -2.75863e-06,
			.c = -0.726263,
		};
		constexpr double pressure_at_stc = 149.326;
		constexpr double tire_load = 298.694;
		constexpr double vehicle_speed = 2.51655;
		constexpr double expected = -1.74866e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 295") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.47091,
			.beta = 6.25627,
			.a = 7.41157,
			.b = 2.12454e-06,
			.c = 0.746517,
		};
		constexpr double pressure_at_stc = 107.697;
		constexpr double tire_load = 2935.74;
		constexpr double vehicle_speed = 80.6091;
		constexpr double expected = 4.74075e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 296") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.717489,
			.beta = 3.79414,
			.a = -7.95902,
			.b = 9.03316e-06,
			.c = -0.712099,
		};
		constexpr double pressure_at_stc = 105.093;
		constexpr double tire_load = 2491.3;
		constexpr double vehicle_speed = 19.1328;
		constexpr double expected = -7.35687e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 297") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.32853,
			.beta = 9.42713,
			.a = 8.44085,
			.b = -6.68768e-06,
			.c = 0.262025,
		};
		constexpr double pressure_at_stc = 140.093;
		constexpr double tire_load = 9534.38;
		constexpr double vehicle_speed = 28.7734;
		constexpr double expected = 1.22194e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 298") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.11496,
			.beta = -3.96032,
			.a = -1.12017,
			.b = -5.79921e-06,
			.c = -0.127075,
		};
		constexpr double pressure_at_stc = 176.108;
		constexpr double tire_load = 9517.05;
		constexpr double vehicle_speed = 61.2862;
		constexpr double expected = -6.22337e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 299") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.52475,
			.beta = 8.73758,
			.a = 2.16662,
			.b = -9.78391e-07,
			.c = 0.326463,
		};
		constexpr double pressure_at_stc = 139.814;
		constexpr double tire_load = 5264.48;
		constexpr double vehicle_speed = 96.6163;
		constexpr double expected = 1.73015e+38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 300") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.95188,
			.beta = 6.47256,
			.a = -6.03922,
			.b = -6.71706e-07,
			.c = 0.61195,
		};
		constexpr double pressure_at_stc = 131.944;
		constexpr double tire_load = 2424;
		constexpr double vehicle_speed = 56.7415;
		constexpr double expected = 1.49519e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 301") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.16091,
			.beta = 8.3464,
			.a = -0.321853,
			.b = -2.14582e-06,
			.c = 0.983115,
		};
		constexpr double pressure_at_stc = 140.279;
		constexpr double tire_load = 7311.09;
		constexpr double vehicle_speed = 27.2331;
		constexpr double expected = 5.41092e+33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 302") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.12109,
			.beta = -5.91525,
			.a = -8.83264,
			.b = -3.77881e-06,
			.c = 0.759287,
		};
		constexpr double pressure_at_stc = 137.249;
		constexpr double tire_load = 8385.63;
		constexpr double vehicle_speed = 45.8432;
		constexpr double expected = 5.13274e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 303") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.88691,
			.beta = 1.18788,
			.a = 4.17111,
			.b = 1.37096e-06,
			.c = -0.581554,
		};
		constexpr double pressure_at_stc = 159.232;
		constexpr double tire_load = 1069.72;
		constexpr double vehicle_speed = 53.6538;
		constexpr double expected = -4.96423e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 304") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.73971,
			.beta = 5.99069,
			.a = 7.43157,
			.b = -5.56794e-06,
			.c = 0.822682,
		};
		constexpr double pressure_at_stc = 119.403;
		constexpr double tire_load = 2834.4;
		constexpr double vehicle_speed = 84.3035;
		constexpr double expected = 2.55061e+35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 305") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.85013,
			.beta = 4.28031,
			.a = -1.30705,
			.b = 2.81365e-06,
			.c = -0.335467,
		};
		constexpr double pressure_at_stc = 168.155;
		constexpr double tire_load = 4582.05;
		constexpr double vehicle_speed = 34.2308;
		constexpr double expected = -1.48809e+30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 306") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.724716,
			.beta = -5.27409,
			.a = 0.37839,
			.b = 6.95455e-06,
			.c = 0.520457,
		};
		constexpr double pressure_at_stc = 176.572;
		constexpr double tire_load = 2945.52;
		constexpr double vehicle_speed = 69.2421;
		constexpr double expected = 3.84317e-16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 307") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.61876,
			.beta = -3.44094,
			.a = 3.46566,
			.b = 4.15854e-06,
			.c = -0.81211,
		};
		constexpr double pressure_at_stc = 159.875;
		constexpr double tire_load = 5293.49;
		constexpr double vehicle_speed = 22.8982;
		constexpr double expected = -2.19815e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 308") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.3652,
			.beta = 4.51634,
			.a = 2.75877,
			.b = 8.18154e-06,
			.c = 0.159214,
		};
		constexpr double pressure_at_stc = 150.849;
		constexpr double tire_load = 7775.5;
		constexpr double vehicle_speed = 58.2802;
		constexpr double expected = 4.38118e+39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 309") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.85823,
			.beta = 1.35801,
			.a = -1.64172,
			.b = 2.52578e-06,
			.c = -0.296531,
		};
		constexpr double pressure_at_stc = 119.355;
		constexpr double tire_load = 4712.78;
		constexpr double vehicle_speed = 90.7278;
		constexpr double expected = -0.250457;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 310") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.26278,
			.beta = 0.947717,
			.a = -5.25391,
			.b = 8.14314e-06,
			.c = 0.22515,
		};
		constexpr double pressure_at_stc = 162.968;
		constexpr double tire_load = 5525.07;
		constexpr double vehicle_speed = 14.188;
		constexpr double expected = 6.62922e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 311") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.24126,
			.beta = 7.1196,
			.a = 9.29501,
			.b = 3.28843e-06,
			.c = -0.899062,
		};
		constexpr double pressure_at_stc = 143.953;
		constexpr double tire_load = 3974.41;
		constexpr double vehicle_speed = 66.7983;
		constexpr double expected = -4.49785e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 312") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.58,
			.beta = -0.576152,
			.a = 9.23671,
			.b = 9.63356e-06,
			.c = 0.135264,
		};
		constexpr double pressure_at_stc = 124.582;
		constexpr double tire_load = 9196.52;
		constexpr double vehicle_speed = 81.0391;
		constexpr double expected = 3.68109e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 313") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.61396,
			.beta = 7.0517,
			.a = 6.73325,
			.b = -6.11173e-06,
			.c = -0.214374,
		};
		constexpr double pressure_at_stc = 134.564;
		constexpr double tire_load = 7924.44;
		constexpr double vehicle_speed = 31.4757;
		constexpr double expected = -1.28887e+21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 314") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.34862,
			.beta = 3.82244,
			.a = 6.77787,
			.b = 6.06247e-06,
			.c = 0.616421,
		};
		constexpr double pressure_at_stc = 176.461;
		constexpr double tire_load = 5592.15;
		constexpr double vehicle_speed = 17.8788;
		constexpr double expected = 1.74964e+34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 315") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.40033,
			.beta = 6.84591,
			.a = 7.07697,
			.b = 6.37653e-06,
			.c = -0.564995,
		};
		constexpr double pressure_at_stc = 168.309;
		constexpr double tire_load = 7586.51;
		constexpr double vehicle_speed = 31.4505;
		constexpr double expected = -2.77737e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 316") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.84636,
			.beta = -0.341952,
			.a = -9.948,
			.b = -1.10466e-06,
			.c = 0.0394408,
		};
		constexpr double pressure_at_stc = 102.526;
		constexpr double tire_load = 8049.61;
		constexpr double vehicle_speed = 50.2739;
		constexpr double expected = 1.06428e-08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 317") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.30664,
			.beta = -6.61918,
			.a = -2.05298,
			.b = 6.05592e-06,
			.c = 0.890822,
		};
		constexpr double pressure_at_stc = 118.738;
		constexpr double tire_load = 6492.41;
		constexpr double vehicle_speed = 52.3865;
		constexpr double expected = 1.88329e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 318") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.01835,
			.beta = 1.63101,
			.a = 4.34793,
			.b = 5.08545e-06,
			.c = 0.634692,
		};
		constexpr double pressure_at_stc = 110.147;
		constexpr double tire_load = 8685.06;
		constexpr double vehicle_speed = 43.4274;
		constexpr double expected = 6.61207e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 319") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.3439,
			.beta = -2.78864,
			.a = 4.49719,
			.b = 8.65884e-06,
			.c = 0.751616,
		};
		constexpr double pressure_at_stc = 106.98;
		constexpr double tire_load = 483.305;
		constexpr double vehicle_speed = 66.5749;
		constexpr double expected = 2.31299e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 320") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.21595,
			.beta = 2.78723,
			.a = 7.73535,
			.b = -7.41474e-06,
			.c = -0.59764,
		};
		constexpr double pressure_at_stc = 191.817;
		constexpr double tire_load = 8770.21;
		constexpr double vehicle_speed = 95.5038;
		constexpr double expected = -5.57874e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 321") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.195011,
			.beta = -3.68744,
			.a = 2.02793,
			.b = 9.02055e-06,
			.c = 0.0170292,
		};
		constexpr double pressure_at_stc = 155.33;
		constexpr double tire_load = 883.921;
		constexpr double vehicle_speed = 98.6726;
		constexpr double expected = 7.85596e-08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 322") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.396453,
			.beta = 1.60281,
			.a = -9.50397,
			.b = -9.14994e-07,
			.c = 0.934828,
		};
		constexpr double pressure_at_stc = 120.637;
		constexpr double tire_load = 279.699;
		constexpr double vehicle_speed = 21.6975;
		constexpr double expected = 3.17851e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 323") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.17832,
			.beta = 9.50981,
			.a = -5.70194,
			.b = 4.74484e-06,
			.c = 0.202761,
		};
		constexpr double pressure_at_stc = 114.092;
		constexpr double tire_load = 418.077;
		constexpr double vehicle_speed = 86.5448;
		constexpr double expected = 9.75503e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 324") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.40199,
			.beta = 7.37295,
			.a = 2.43843,
			.b = 3.47103e-06,
			.c = 0.568411,
		};
		constexpr double pressure_at_stc = 109.328;
		constexpr double tire_load = 6516.28;
		constexpr double vehicle_speed = 67.2675;
		constexpr double expected = 4.95744e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 325") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.58305,
			.beta = 3.24687,
			.a = 1.61453,
			.b = 8.08479e-06,
			.c = 0.5026,
		};
		constexpr double pressure_at_stc = 164.326;
		constexpr double tire_load = 112.636;
		constexpr double vehicle_speed = 70.6975;
		constexpr double expected = 3.50412e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 326") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.74291,
			.beta = -4.71336,
			.a = -2.9008,
			.b = 3.61198e-06,
			.c = 0.275587,
		};
		constexpr double pressure_at_stc = 125.2;
		constexpr double tire_load = 5832.56;
		constexpr double vehicle_speed = 98.5196;
		constexpr double expected = 1.68569e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 327") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.911628,
			.beta = -1.39343,
			.a = 9.76706,
			.b = 6.01565e-06,
			.c = 0.95461,
		};
		constexpr double pressure_at_stc = 133.254;
		constexpr double tire_load = 2530.85;
		constexpr double vehicle_speed = 85.6206;
		constexpr double expected = 0.0189884;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 328") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.27277,
			.beta = -3.96921,
			.a = -1.07251,
			.b = -5.13073e-06,
			.c = -0.640155,
		};
		constexpr double pressure_at_stc = 135.231;
		constexpr double tire_load = 548.092;
		constexpr double vehicle_speed = 19.4626;
		constexpr double expected = -2.4396e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 329") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.307809,
			.beta = -9.04041,
			.a = 0.720825,
			.b = 9.54634e-06,
			.c = 0.525538,
		};
		constexpr double pressure_at_stc = 127.036;
		constexpr double tire_load = 4957.52;
		constexpr double vehicle_speed = 77.388;
		constexpr double expected = 7.10345e-29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 330") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.73097,
			.beta = -8.07283,
			.a = -7.19985,
			.b = 5.43077e-06,
			.c = -0.573177,
		};
		constexpr double pressure_at_stc = 175.143;
		constexpr double tire_load = 1360.56;
		constexpr double vehicle_speed = 81.6693;
		constexpr double expected = -6.45825e-41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 331") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.96001,
			.beta = 8.52231,
			.a = 8.38666,
			.b = -7.67672e-06,
			.c = -0.277499,
		};
		constexpr double pressure_at_stc = 115.481;
		constexpr double tire_load = 8998.8;
		constexpr double vehicle_speed = 82.6033;
		constexpr double expected = -2.08242e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 332") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.66717,
			.beta = 7.6434,
			.a = 6.71507,
			.b = -4.22045e-06,
			.c = -0.0148218,
		};
		constexpr double pressure_at_stc = 100.684;
		constexpr double tire_load = 2107.19;
		constexpr double vehicle_speed = 87.7133;
		constexpr double expected = -8.60999e+47;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 333") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.60184,
			.beta = -5.73971,
			.a = 6.33617,
			.b = -5.23316e-06,
			.c = -0.541169,
		};
		constexpr double pressure_at_stc = 121.042;
		constexpr double tire_load = 3283.5;
		constexpr double vehicle_speed = 0.604275;
		constexpr double expected = 1.39764e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 334") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.75677,
			.beta = 8.65377,
			.a = 3.90325,
			.b = -4.68079e-07,
			.c = -0.956301,
		};
		constexpr double pressure_at_stc = 118.532;
		constexpr double tire_load = 9305.94;
		constexpr double vehicle_speed = 13.5221;
		constexpr double expected = -4.86239e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 335") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.6288,
			.beta = -6.56735,
			.a = -1.25012,
			.b = 7.62387e-06,
			.c = -0.847998,
		};
		constexpr double pressure_at_stc = 129.858;
		constexpr double tire_load = 1254.2;
		constexpr double vehicle_speed = 73.9813;
		constexpr double expected = -2.63906e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 336") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.36203,
			.beta = 2.76316,
			.a = 7.96936,
			.b = 4.04171e-06,
			.c = -0.365554,
		};
		constexpr double pressure_at_stc = 142.181;
		constexpr double tire_load = 2915.2;
		constexpr double vehicle_speed = 19.1829;
		constexpr double expected = -5.34258e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 337") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.37073,
			.beta = 2.53687,
			.a = -2.05148,
			.b = 2.20084e-06,
			.c = -0.854713,
		};
		constexpr double pressure_at_stc = 148.041;
		constexpr double tire_load = 1997.25;
		constexpr double vehicle_speed = 93.5783;
		constexpr double expected = -3.36695e+31;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 338") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.38202,
			.beta = -6.28588,
			.a = -3.02236,
			.b = -9.76865e-07,
			.c = 0.765281,
		};
		constexpr double pressure_at_stc = 132.316;
		constexpr double tire_load = 5196.62;
		constexpr double vehicle_speed = 86.3285;
		constexpr double expected = 2.87352e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 339") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.525486,
			.beta = -1.62449,
			.a = -7.67158,
			.b = -1.88249e-06,
			.c = -0.269263,
		};
		constexpr double pressure_at_stc = 108.69;
		constexpr double tire_load = 6034.28;
		constexpr double vehicle_speed = 13.4081;
		constexpr double expected = -3.90144e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 340") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.85927,
			.beta = 9.00693,
			.a = -9.75881,
			.b = 6.9076e-06,
			.c = -0.0453602,
		};
		constexpr double pressure_at_stc = 107.396;
		constexpr double tire_load = 7431.4;
		constexpr double vehicle_speed = 91.73;
		constexpr double expected = -3.10983e+52;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 341") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.92481,
			.beta = -1.66231,
			.a = 9.94712,
			.b = 3.17416e-06,
			.c = 0.829343,
		};
		constexpr double pressure_at_stc = 192.79;
		constexpr double tire_load = 3708.63;
		constexpr double vehicle_speed = 56.5759;
		constexpr double expected = 9.94351e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 342") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.9975,
			.beta = -3.31437,
			.a = 9.56654,
			.b = -3.39283e-06,
			.c = -0.24927,
		};
		constexpr double pressure_at_stc = 189.267;
		constexpr double tire_load = 4134.52;
		constexpr double vehicle_speed = 2.21728;
		constexpr double expected = -1.05961e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 343") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.485058,
			.beta = 6.62522,
			.a = -9.34829,
			.b = 3.72949e-06,
			.c = -0.219776,
		};
		constexpr double pressure_at_stc = 125.031;
		constexpr double tire_load = 2596.64;
		constexpr double vehicle_speed = 12.869;
		constexpr double expected = -2.09213e+26;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 344") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.52732,
			.beta = -0.887306,
			.a = -0.344849,
			.b = 4.81475e-06,
			.c = -0.887556,
		};
		constexpr double pressure_at_stc = 127.97;
		constexpr double tire_load = 877.272;
		constexpr double vehicle_speed = 75.6129;
		constexpr double expected = -265930;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 345") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.48335,
			.beta = 2.14812,
			.a = -4.40426,
			.b = -2.81139e-06,
			.c = 0.248931,
		};
		constexpr double pressure_at_stc = 110.926;
		constexpr double tire_load = 3807.91;
		constexpr double vehicle_speed = 96.8933;
		constexpr double expected = 1.78432e+17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 346") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.9326,
			.beta = -5.25112,
			.a = 0.834284,
			.b = -9.70054e-06,
			.c = 0.225188,
		};
		constexpr double pressure_at_stc = 116.604;
		constexpr double tire_load = 3291.96;
		constexpr double vehicle_speed = 11.8356;
		constexpr double expected = 400.838;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 347") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.98595,
			.beta = 3.49211,
			.a = -4.72082,
			.b = 3.16175e-06,
			.c = -0.835288,
		};
		constexpr double pressure_at_stc = 199.984;
		constexpr double tire_load = 3603.42;
		constexpr double vehicle_speed = 1.57946;
		constexpr double expected = -1.24019e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 348") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.673886,
			.beta = 5.87157,
			.a = -4.00658,
			.b = -8.65907e-06,
			.c = -0.563718,
		};
		constexpr double pressure_at_stc = 139.745;
		constexpr double tire_load = 8327.84;
		constexpr double vehicle_speed = 33.43;
		constexpr double expected = -3.06291e+25;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 349") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.13588,
			.beta = -5.87171,
			.a = -2.56664,
			.b = -1.27822e-06,
			.c = 0.535689,
		};
		constexpr double pressure_at_stc = 199.21;
		constexpr double tire_load = 4916.55;
		constexpr double vehicle_speed = 3.26133;
		constexpr double expected = 0.0764748;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 350") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.77381,
			.beta = 7.45036,
			.a = -1.57304,
			.b = -3.9217e-06,
			.c = -0.531692,
		};
		constexpr double pressure_at_stc = 117.401;
		constexpr double tire_load = 2103.35;
		constexpr double vehicle_speed = 43.9134;
		constexpr double expected = -5.7629e+38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 351") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.16289,
			.beta = -3.13235,
			.a = -5.55847,
			.b = 1.77741e-07,
			.c = 0.395632,
		};
		constexpr double pressure_at_stc = 199.115;
		constexpr double tire_load = 7950.11;
		constexpr double vehicle_speed = 41.364;
		constexpr double expected = 7.16969e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 352") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.6239,
			.beta = 2.07335,
			.a = -1.74663,
			.b = -2.74078e-06,
			.c = 0.820957,
		};
		constexpr double pressure_at_stc = 197.61;
		constexpr double tire_load = 7899.7;
		constexpr double vehicle_speed = 99.4645;
		constexpr double expected = 60756.3;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 353") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.89888,
			.beta = -9.18745,
			.a = 3.27206,
			.b = 9.04717e-06,
			.c = 0.216042,
		};
		constexpr double pressure_at_stc = 112.441;
		constexpr double tire_load = 3601.36;
		constexpr double vehicle_speed = 31.9067;
		constexpr double expected = 3.80662e-46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 354") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.54315,
			.beta = 4.03993,
			.a = -7.48566,
			.b = 4.63396e-06,
			.c = -0.124663,
		};
		constexpr double pressure_at_stc = 100.399;
		constexpr double tire_load = 9474.84;
		constexpr double vehicle_speed = 79.5054;
		constexpr double expected = -94447.8;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 355") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.01736,
			.beta = -5.24761,
			.a = -9.39281,
			.b = -7.57265e-06,
			.c = -0.928762,
		};
		constexpr double pressure_at_stc = 190.896;
		constexpr double tire_load = 3512.04;
		constexpr double vehicle_speed = 84.7447;
		constexpr double expected = -1.47327e-23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 356") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.74331,
			.beta = 5.95662,
			.a = 2.92897,
			.b = 3.15859e-07,
			.c = 0.770903,
		};
		constexpr double pressure_at_stc = 124.37;
		constexpr double tire_load = 5455.21;
		constexpr double vehicle_speed = 49.5121;
		constexpr double expected = 2.15318e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 357") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.403415,
			.beta = 2.89649,
			.a = 3.57358,
			.b = 9.08531e-06,
			.c = -0.0130317,
		};
		constexpr double pressure_at_stc = 174.307;
		constexpr double tire_load = 7609.66;
		constexpr double vehicle_speed = 91.233;
		constexpr double expected = -1.96477e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 358") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.84613,
			.beta = -5.46201,
			.a = 1.7632,
			.b = -1.71313e-06,
			.c = -0.445617,
		};
		constexpr double pressure_at_stc = 104.282;
		constexpr double tire_load = 491.553;
		constexpr double vehicle_speed = 18.6069;
		constexpr double expected = -6.31588e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 359") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.01718,
			.beta = 3.44199,
			.a = -6.94565,
			.b = 9.66925e-06,
			.c = 0.174839,
		};
		constexpr double pressure_at_stc = 163.608;
		constexpr double tire_load = 5892.05;
		constexpr double vehicle_speed = 96.8625;
		constexpr double expected = 1.84881e+37;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 360") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.3275,
			.beta = 9.90114,
			.a = 3.23284,
			.b = -8.24595e-06,
			.c = 0.260335,
		};
		constexpr double pressure_at_stc = 113.862;
		constexpr double tire_load = 9944.51;
		constexpr double vehicle_speed = 72.2633;
		constexpr double expected = 3.60193e+46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 361") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.63992,
			.beta = 6.36897,
			.a = 5.77282,
			.b = -6.27775e-06,
			.c = -0.23695,
		};
		constexpr double pressure_at_stc = 141.995;
		constexpr double tire_load = 8482.04;
		constexpr double vehicle_speed = 72.1743;
		constexpr double expected = -4.65099e+45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 362") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.36661,
			.beta = -6.77199,
			.a = -8.85057,
			.b = 9.84231e-06,
			.c = 0.829142,
		};
		constexpr double pressure_at_stc = 115.898;
		constexpr double tire_load = 4167.64;
		constexpr double vehicle_speed = 80.7117;
		constexpr double expected = 9.94979e-40;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 363") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.05789,
			.beta = -4.69589,
			.a = 8.06827,
			.b = -5.37391e-06,
			.c = 0.732751,
		};
		constexpr double pressure_at_stc = 117.632;
		constexpr double tire_load = 116.318;
		constexpr double vehicle_speed = 0.813956;
		constexpr double expected = 1.62813e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 364") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.21813,
			.beta = -3.21855,
			.a = 3.84827,
			.b = -4.69163e-06,
			.c = -0.20679,
		};
		constexpr double pressure_at_stc = 196.565;
		constexpr double tire_load = 2577.67;
		constexpr double vehicle_speed = 87.3904;
		constexpr double expected = -8.93236e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 365") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.12519,
			.beta = 0.036596,
			.a = -6.77219,
			.b = 7.81918e-06,
			.c = -0.239727,
		};
		constexpr double pressure_at_stc = 145.863;
		constexpr double tire_load = 4872.91;
		constexpr double vehicle_speed = 66.5172;
		constexpr double expected = -1.0466e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 366") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.10619,
			.beta = 1.06926,
			.a = -5.87604,
			.b = 9.18444e-07,
			.c = -0.730225,
		};
		constexpr double pressure_at_stc = 161.597;
		constexpr double tire_load = 3557.32;
		constexpr double vehicle_speed = 16.9011;
		constexpr double expected = -2.12776e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 367") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.22624,
			.beta = 1.38357,
			.a = -5.92346,
			.b = -3.41014e-06,
			.c = -0.0271312,
		};
		constexpr double pressure_at_stc = 173.113;
		constexpr double tire_load = 1177.34;
		constexpr double vehicle_speed = 65.6597;
		constexpr double expected = -4.03521e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 368") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.08376,
			.beta = 9.72308,
			.a = 7.24849,
			.b = 7.31375e-06,
			.c = 0.357664,
		};
		constexpr double pressure_at_stc = 156.242;
		constexpr double tire_load = 4778.12;
		constexpr double vehicle_speed = 19.4825;
		constexpr double expected = 3.9102e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 369") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.02613,
			.beta = 8.10692,
			.a = -9.18552,
			.b = 6.10638e-06,
			.c = -0.0529992,
		};
		constexpr double pressure_at_stc = 180.121;
		constexpr double tire_load = 1247.99;
		constexpr double vehicle_speed = 15.6261;
		constexpr double expected = -3.33302e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 370") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.36233,
			.beta = -5.35478,
			.a = -9.34487,
			.b = 4.37778e-06,
			.c = -0.738213,
		};
		constexpr double pressure_at_stc = 127.374;
		constexpr double tire_load = 4979.04;
		constexpr double vehicle_speed = 93.5159;
		constexpr double expected = -2.61058e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 371") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.941605,
			.beta = -3.8135,
			.a = 5.29244,
			.b = 8.69871e-06,
			.c = 0.132224,
		};
		constexpr double pressure_at_stc = 173.781;
		constexpr double tire_load = 6211.64;
		constexpr double vehicle_speed = 84.9499;
		constexpr double expected = 3.29472e-13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 372") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.48012,
			.beta = 8.71238,
			.a = -7.59697,
			.b = 2.72388e-06,
			.c = 0.400209,
		};
		constexpr double pressure_at_stc = 169.151;
		constexpr double tire_load = 5652.19;
		constexpr double vehicle_speed = 28.0231;
		constexpr double expected = 1.56997e+55;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 373") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.42505,
			.beta = 4.40703,
			.a = -9.88536,
			.b = -5.71307e-06,
			.c = -0.337857,
		};
		constexpr double pressure_at_stc = 166.832;
		constexpr double tire_load = 3191.94;
		constexpr double vehicle_speed = 85.6914;
		constexpr double expected = -6.07479e+29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 374") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.42891,
			.beta = -7.67551,
			.a = 7.59916,
			.b = 6.96072e-06,
			.c = 0.694372,
		};
		constexpr double pressure_at_stc = 111.603;
		constexpr double tire_load = 8384.3;
		constexpr double vehicle_speed = 77.5561;
		constexpr double expected = 3.95886e-33;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 375") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.8046,
			.beta = -0.239682,
			.a = -8.29934,
			.b = 1.30446e-06,
			.c = 0.878764,
		};
		constexpr double pressure_at_stc = 172.057;
		constexpr double tire_load = 3670.84;
		constexpr double vehicle_speed = 70.8834;
		constexpr double expected = 2.24712e+21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 376") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.03,
			.beta = -2.62434,
			.a = -5.08734,
			.b = 3.30045e-06,
			.c = 0.41269,
		};
		constexpr double pressure_at_stc = 137.836;
		constexpr double tire_load = 2962.24;
		constexpr double vehicle_speed = 69.8648;
		constexpr double expected = 2.2164e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 377") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.31983,
			.beta = -3.04542,
			.a = -6.48617,
			.b = 7.07771e-06,
			.c = -0.832967,
		};
		constexpr double pressure_at_stc = 164.731;
		constexpr double tire_load = 6294.9;
		constexpr double vehicle_speed = 38.1495;
		constexpr double expected = -7.13316e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 378") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.63716,
			.beta = -6.43175,
			.a = -1.84421,
			.b = -8.68007e-06,
			.c = 0.489941,
		};
		constexpr double pressure_at_stc = 170.638;
		constexpr double tire_load = 4796.82;
		constexpr double vehicle_speed = 3.51582;
		constexpr double expected = 1.45473e-39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 379") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.64192,
			.beta = -5.09001,
			.a = 0.831709,
			.b = -9.13547e-06,
			.c = 0.0250578,
		};
		constexpr double pressure_at_stc = 180.237;
		constexpr double tire_load = 6233.47;
		constexpr double vehicle_speed = 60.52;
		constexpr double expected = 1.8434e-36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 380") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.72149,
			.beta = -3.23282,
			.a = -0.13461,
			.b = -5.5997e-06,
			.c = 0.124447,
		};
		constexpr double pressure_at_stc = 108.232;
		constexpr double tire_load = 9473.93;
		constexpr double vehicle_speed = 9.42938;
		constexpr double expected = 1.10501e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 381") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.25025,
			.beta = 9.09082,
			.a = -2.66306,
			.b = 7.82264e-06,
			.c = -0.919907,
		};
		constexpr double pressure_at_stc = 107.891;
		constexpr double tire_load = 4265.5;
		constexpr double vehicle_speed = 55.061;
		constexpr double expected = -1.7027e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 382") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.05747,
			.beta = -3.95882,
			.a = 3.4895,
			.b = 7.70053e-06,
			.c = -0.43603,
		};
		constexpr double pressure_at_stc = 111.806;
		constexpr double tire_load = 541.907;
		constexpr double vehicle_speed = 45.0659;
		constexpr double expected = -35.332;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 383") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.00575,
			.beta = -3.82493,
			.a = 9.55575,
			.b = -8.19011e-06,
			.c = -0.495265,
		};
		constexpr double pressure_at_stc = 175.083;
		constexpr double tire_load = 8292.92;
		constexpr double vehicle_speed = 53.1142;
		constexpr double expected = -1.68923e+07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 384") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.89265,
			.beta = 3.36282,
			.a = 3.61899,
			.b = -5.47731e-06,
			.c = -0.995917,
		};
		constexpr double pressure_at_stc = 107.057;
		constexpr double tire_load = 4340.84;
		constexpr double vehicle_speed = 88.3741;
		constexpr double expected = -1.56964e+29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 385") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.11355,
			.beta = -4.36867,
			.a = -0.60727,
			.b = 6.79805e-06,
			.c = 0.543712,
		};
		constexpr double pressure_at_stc = 122.106;
		constexpr double tire_load = 9342.62;
		constexpr double vehicle_speed = 83.5245;
		constexpr double expected = 154.889;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 386") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.42441,
			.beta = 4.59446,
			.a = 6.46712,
			.b = -1.32274e-06,
			.c = 0.083969,
		};
		constexpr double pressure_at_stc = 133.327;
		constexpr double tire_load = 9984.65;
		constexpr double vehicle_speed = 4.35332;
		constexpr double expected = 9.10489e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 387") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.93149,
			.beta = 6.97749,
			.a = 3.02232,
			.b = -5.3038e-06,
			.c = 0.284473,
		};
		constexpr double pressure_at_stc = 163.585;
		constexpr double tire_load = 5249.47;
		constexpr double vehicle_speed = 53.9804;
		constexpr double expected = 3.00529e+36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 388") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.04304,
			.beta = 2.30188,
			.a = 5.15696,
			.b = 4.28293e-06,
			.c = 0.864326,
		};
		constexpr double pressure_at_stc = 191.221;
		constexpr double tire_load = 7505.08;
		constexpr double vehicle_speed = 75.0353;
		constexpr double expected = 5.99336e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 389") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.18739,
			.beta = -5.485,
			.a = -0.220371,
			.b = -2.80666e-06,
			.c = -0.667837,
		};
		constexpr double pressure_at_stc = 167.559;
		constexpr double tire_load = 3780.55;
		constexpr double vehicle_speed = 43.764;
		constexpr double expected = -1.45325e-36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 390") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.51554,
			.beta = -8.61093,
			.a = -3.09745,
			.b = 5.90704e-06,
			.c = -0.177275,
		};
		constexpr double pressure_at_stc = 116.129;
		constexpr double tire_load = 2500.81;
		constexpr double vehicle_speed = 56.916;
		constexpr double expected = -7.42432e-19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 391") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.00573,
			.beta = -5.32358,
			.a = -3.51584,
			.b = 1.71973e-06,
			.c = -0.701138,
		};
		constexpr double pressure_at_stc = 119.156;
		constexpr double tire_load = 1815.18;
		constexpr double vehicle_speed = 83.4853;
		constexpr double expected = -5.69641e-32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 392") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.3911,
			.beta = 8.02698,
			.a = -9.64972,
			.b = 9.12248e-06,
			.c = -0.779437,
		};
		constexpr double pressure_at_stc = 122.749;
		constexpr double tire_load = 4145.31;
		constexpr double vehicle_speed = 86.1674;
		constexpr double expected = -2.97181e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 393") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.59728,
			.beta = 5.91822,
			.a = 0.257519,
			.b = -2.8201e-06,
			.c = -0.513949,
		};
		constexpr double pressure_at_stc = 162.85;
		constexpr double tire_load = 6994.85;
		constexpr double vehicle_speed = 11.2817;
		constexpr double expected = -1.23205e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 394") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.91142,
			.beta = 6.78815,
			.a = -4.02454,
			.b = 7.26182e-06,
			.c = -0.537978,
		};
		constexpr double pressure_at_stc = 135.301;
		constexpr double tire_load = 5027.39;
		constexpr double vehicle_speed = 35.9205;
		constexpr double expected = -1.22076e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 395") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.13749,
			.beta = -2.09419,
			.a = 7.95645,
			.b = 1.29645e-06,
			.c = -0.876998,
		};
		constexpr double pressure_at_stc = 196.885;
		constexpr double tire_load = 1735.15;
		constexpr double vehicle_speed = 65.2932;
		constexpr double expected = -1.70747e-21;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 396") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.55203,
			.beta = 7.28086,
			.a = 9.31973,
			.b = 4.27281e-06,
			.c = 0.375029,
		};
		constexpr double pressure_at_stc = 120.204;
		constexpr double tire_load = 8319.26;
		constexpr double vehicle_speed = 70.2424;
		constexpr double expected = 1.41158e+36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 397") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.31283,
			.beta = -1.93643,
			.a = -1.18184,
			.b = 1.73882e-06,
			.c = -0.241127,
		};
		constexpr double pressure_at_stc = 102.878;
		constexpr double tire_load = 3793.98;
		constexpr double vehicle_speed = 61.6463;
		constexpr double expected = -3.17842e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 398") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.795773,
			.beta = 4.2629,
			.a = 3.38488,
			.b = 2.07696e-07,
			.c = 0.740631,
		};
		constexpr double pressure_at_stc = 125.001;
		constexpr double tire_load = 7440.62;
		constexpr double vehicle_speed = 83.9089;
		constexpr double expected = 4.62873e+19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 399") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.3123,
			.beta = -2.46686,
			.a = 5.21878,
			.b = 2.18984e-06,
			.c = 0.514334,
		};
		constexpr double pressure_at_stc = 138.661;
		constexpr double tire_load = 824.559;
		constexpr double vehicle_speed = 4.28364;
		constexpr double expected = 6.56035e-13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 400") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.96566,
			.beta = 4.7111,
			.a = -2.8125,
			.b = -4.18974e-06,
			.c = 0.560598,
		};
		constexpr double pressure_at_stc = 151.79;
		constexpr double tire_load = 7401.24;
		constexpr double vehicle_speed = 57.2559;
		constexpr double expected = 1.18676e+29;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 401") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.77587,
			.beta = 8.66787,
			.a = -6.61506,
			.b = -5.48478e-06,
			.c = -0.421045,
		};
		constexpr double pressure_at_stc = 149.615;
		constexpr double tire_load = 6211.18;
		constexpr double vehicle_speed = 75.9307;
		constexpr double expected = -2.90999e+56;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 402") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.37545,
			.beta = 5.29857,
			.a = 8.60376,
			.b = -1.06886e-06,
			.c = 0.451213,
		};
		constexpr double pressure_at_stc = 110.108;
		constexpr double tire_load = 6778.71;
		constexpr double vehicle_speed = 83.2822;
		constexpr double expected = 1.03685e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 403") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.436552,
			.beta = 6.71334,
			.a = -2.27576,
			.b = 6.07811e-07,
			.c = -0.641732,
		};
		constexpr double pressure_at_stc = 172.411;
		constexpr double tire_load = 9028.35;
		constexpr double vehicle_speed = 29.2397;
		constexpr double expected = -2.41983e+31;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 404") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.05959,
			.beta = -1.82014,
			.a = 3.37748,
			.b = -4.13937e-06,
			.c = -0.0573504,
		};
		constexpr double pressure_at_stc = 146.871;
		constexpr double tire_load = 6583.42;
		constexpr double vehicle_speed = 5.15758;
		constexpr double expected = -6.33227e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 405") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.98623,
			.beta = 9.05986,
			.a = -8.63603,
			.b = -5.35994e-06,
			.c = -0.824461,
		};
		constexpr double pressure_at_stc = 120.225;
		constexpr double tire_load = 4076.04;
		constexpr double vehicle_speed = 38.5192;
		constexpr double expected = -4.1425e+28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 406") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.620786,
			.beta = 9.91486,
			.a = -5.8033,
			.b = -8.8826e-06,
			.c = -0.828138,
		};
		constexpr double pressure_at_stc = 185.228;
		constexpr double tire_load = 7088.04;
		constexpr double vehicle_speed = 54.3918;
		constexpr double expected = -1.86864e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 407") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.0271371,
			.beta = 5.66711,
			.a = 4.78405,
			.b = 4.82032e-06,
			.c = 0.633093,
		};
		constexpr double pressure_at_stc = 120.043;
		constexpr double tire_load = 2401.9;
		constexpr double vehicle_speed = 12.6281;
		constexpr double expected = 2.15144e+22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 408") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.65863,
			.beta = -5.82407,
			.a = 6.45245,
			.b = -2.23596e-06,
			.c = -0.469946,
		};
		constexpr double pressure_at_stc = 117.415;
		constexpr double tire_load = 8180.04;
		constexpr double vehicle_speed = 98.9097;
		constexpr double expected = -2.2123e-28;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 409") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.40741,
			.beta = -4.7371,
			.a = -7.05564,
			.b = -1.88941e-06,
			.c = 0.202707,
		};
		constexpr double pressure_at_stc = 122.893;
		constexpr double tire_load = 7061.86;
		constexpr double vehicle_speed = 12.342;
		constexpr double expected = 6.22726e-34;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 410") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.18023,
			.beta = 7.75789,
			.a = -9.53966,
			.b = 6.72609e-06,
			.c = 0.910183,
		};
		constexpr double pressure_at_stc = 145.237;
		constexpr double tire_load = 7735.36;
		constexpr double vehicle_speed = 87.6216;
		constexpr double expected = 3.72881e+32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 411") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.93666,
			.beta = 2.13617,
			.a = -7.20478,
			.b = 2.51972e-06,
			.c = 0.793146,
		};
		constexpr double pressure_at_stc = 139.424;
		constexpr double tire_load = 5835.12;
		constexpr double vehicle_speed = 2.20823;
		constexpr double expected = 334759;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 412") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.61275,
			.beta = -2.54408,
			.a = -8.40407,
			.b = -1.65701e-06,
			.c = -0.979128,
		};
		constexpr double pressure_at_stc = 179.399;
		constexpr double tire_load = 3872.12;
		constexpr double vehicle_speed = 81.9135;
		constexpr double expected = -8803.39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 413") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.681865,
			.beta = 6.7518,
			.a = 9.597,
			.b = -3.88353e-06,
			.c = 0.525466,
		};
		constexpr double pressure_at_stc = 125.577;
		constexpr double tire_load = 2683.43;
		constexpr double vehicle_speed = 8.41929;
		constexpr double expected = 2.57588e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 414") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.94497,
			.beta = -3.03188,
			.a = -8.78211,
			.b = 1.13441e-06,
			.c = 0.0373286,
		};
		constexpr double pressure_at_stc = 144.059;
		constexpr double tire_load = 4567.99;
		constexpr double vehicle_speed = 86.4902;
		constexpr double expected = 9.88724e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 415") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.35336,
			.beta = -1.85474,
			.a = 5.79541,
			.b = -7.03896e-06,
			.c = 0.420336,
		};
		constexpr double pressure_at_stc = 139.746;
		constexpr double tire_load = 9304.11;
		constexpr double vehicle_speed = 42.3573;
		constexpr double expected = 3.8082e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 416") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.86179,
			.beta = 9.40336,
			.a = 2.22092,
			.b = 2.55234e-06,
			.c = -0.828883,
		};
		constexpr double pressure_at_stc = 128.396;
		constexpr double tire_load = 6632.82;
		constexpr double vehicle_speed = 47.0541;
		constexpr double expected = -4.7092e+52;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 417") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.47331,
			.beta = -9.40382,
			.a = 2.76689,
			.b = 2.6069e-06,
			.c = 0.694472,
		};
		constexpr double pressure_at_stc = 147.952;
		constexpr double tire_load = 935.969;
		constexpr double vehicle_speed = 91.9665;
		constexpr double expected = 1.15499e-35;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 418") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.939229,
			.beta = 2.75752,
			.a = -6.4781,
			.b = -2.32399e-07,
			.c = 0.481944,
		};
		constexpr double pressure_at_stc = 194.614;
		constexpr double tire_load = 8536.86;
		constexpr double vehicle_speed = 24.9411;
		constexpr double expected = 1.90229e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 419") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.67407,
			.beta = 6.83396,
			.a = 7.2913,
			.b = 3.0237e-06,
			.c = 0.7753,
		};
		constexpr double pressure_at_stc = 162.597;
		constexpr double tire_load = 4773.82;
		constexpr double vehicle_speed = 24.3386;
		constexpr double expected = 2.02706e+50;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 420") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.98506,
			.beta = -7.01621,
			.a = -1.17224,
			.b = 7.98554e-06,
			.c = -0.58299,
		};
		constexpr double pressure_at_stc = 105.26;
		constexpr double tire_load = 4503.78;
		constexpr double vehicle_speed = 72.9914;
		constexpr double expected = -8.16086e-30;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 421") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.52214,
			.beta = -2.48924,
			.a = -3.22429,
			.b = -8.49377e-06,
			.c = 0.976932,
		};
		constexpr double pressure_at_stc = 137.749;
		constexpr double tire_load = 796.276;
		constexpr double vehicle_speed = 90.5956;
		constexpr double expected = 2.51273e-08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 422") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.12271,
			.beta = 7.11472,
			.a = -6.81903,
			.b = 6.39368e-06,
			.c = 0.232999,
		};
		constexpr double pressure_at_stc = 177.798;
		constexpr double tire_load = 1209.3;
		constexpr double vehicle_speed = 29.741;
		constexpr double expected = 3.81052e+20;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 423") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.20663,
			.beta = -2.04429,
			.a = 8.22808,
			.b = -3.59882e-07,
			.c = 0.24833,
		};
		constexpr double pressure_at_stc = 195.714;
		constexpr double tire_load = 4358.87;
		constexpr double vehicle_speed = 98.2658;
		constexpr double expected = 1.32123e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 424") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.00742,
			.beta = -7.27012,
			.a = 2.16541,
			.b = 5.85587e-07,
			.c = -0.897398,
		};
		constexpr double pressure_at_stc = 150.56;
		constexpr double tire_load = 9946.71;
		constexpr double vehicle_speed = 62.4667;
		constexpr double expected = -9.49991e-45;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 425") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.74309,
			.beta = 0.415101,
			.a = -8.521,
			.b = -9.84006e-06,
			.c = 0.451945,
		};
		constexpr double pressure_at_stc = 154.014;
		constexpr double tire_load = 9804.02;
		constexpr double vehicle_speed = 52.8083;
		constexpr double expected = 1.52512e+27;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 426") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.17086,
			.beta = 4.38058,
			.a = 1.34987,
			.b = 6.21086e-06,
			.c = 0.260124,
		};
		constexpr double pressure_at_stc = 102.299;
		constexpr double tire_load = 2937.55;
		constexpr double vehicle_speed = 6.56442;
		constexpr double expected = 882.007;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 427") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.08448,
			.beta = -8.0471,
			.a = 0.830744,
			.b = -6.41594e-06,
			.c = 0.863621,
		};
		constexpr double pressure_at_stc = 183.083;
		constexpr double tire_load = 53.46;
		constexpr double vehicle_speed = 40.8519;
		constexpr double expected = 1.20833e-05;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 428") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.80858,
			.beta = 7.4201,
			.a = 7.59131,
			.b = -6.81644e-06,
			.c = 0.410565,
		};
		constexpr double pressure_at_stc = 132.817;
		constexpr double tire_load = 4229.97;
		constexpr double vehicle_speed = 88.6982;
		constexpr double expected = 2.77398e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 429") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.05481,
			.beta = -5.47459,
			.a = 1.38519,
			.b = 9.44291e-06,
			.c = 0.477852,
		};
		constexpr double pressure_at_stc = 108.858;
		constexpr double tire_load = 9305.77;
		constexpr double vehicle_speed = 81.3634;
		constexpr double expected = 0.00179948;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 430") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.33294,
			.beta = 3.88945,
			.a = 2.47762,
			.b = -4.86386e-06,
			.c = 0.460638,
		};
		constexpr double pressure_at_stc = 158.78;
		constexpr double tire_load = 190.547;
		constexpr double vehicle_speed = 67.7909;
		constexpr double expected = 4.41976e+31;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 431") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.54263,
			.beta = -0.661795,
			.a = 8.04121,
			.b = 6.2872e-06,
			.c = 0.045192,
		};
		constexpr double pressure_at_stc = 184.072;
		constexpr double tire_load = 4401.57;
		constexpr double vehicle_speed = 82.7785;
		constexpr double expected = 8.01759e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 432") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.586,
			.beta = -6.14648,
			.a = -0.426669,
			.b = 7.9568e-06,
			.c = -0.427946,
		};
		constexpr double pressure_at_stc = 192.648;
		constexpr double tire_load = 2045.36;
		constexpr double vehicle_speed = 12.7271;
		constexpr double expected = -6.26928e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 433") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.52094,
			.beta = 0.910992,
			.a = -8.92076,
			.b = -1.27244e-06,
			.c = 0.387851,
		};
		constexpr double pressure_at_stc = 155.538;
		constexpr double tire_load = 256.352;
		constexpr double vehicle_speed = 95.1119;
		constexpr double expected = 2.33029e-10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 434") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.74671,
			.beta = 4.43634,
			.a = -9.12875,
			.b = 1.55565e-06,
			.c = -0.81984,
		};
		constexpr double pressure_at_stc = 167.646;
		constexpr double tire_load = 7019.03;
		constexpr double vehicle_speed = 60.6534;
		constexpr double expected = -7.71362e+38;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 435") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.05693,
			.beta = 3.64793,
			.a = 2.18951,
			.b = 7.25208e-06,
			.c = -0.835668,
		};
		constexpr double pressure_at_stc = 186.514;
		constexpr double tire_load = 834.131;
		constexpr double vehicle_speed = 75.923;
		constexpr double expected = -3.23909e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 436") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.290497,
			.beta = -2.95116,
			.a = -3.38641,
			.b = -5.97875e-06,
			.c = 0.286774,
		};
		constexpr double pressure_at_stc = 147.217;
		constexpr double tire_load = 6081.18;
		constexpr double vehicle_speed = 36.2291;
		constexpr double expected = 1.41447e-07;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 437") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.21765,
			.beta = 6.21366,
			.a = -8.54873,
			.b = -6.06353e-06,
			.c = 0.864998,
		};
		constexpr double pressure_at_stc = 163.698;
		constexpr double tire_load = 7972.99;
		constexpr double vehicle_speed = 48.7168;
		constexpr double expected = 5.73059e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 438") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.5076,
			.beta = 2.54709,
			.a = -0.679003,
			.b = 4.5251e-06,
			.c = -0.0957173,
		};
		constexpr double pressure_at_stc = 197.038;
		constexpr double tire_load = 2933.26;
		constexpr double vehicle_speed = 5.08924;
		constexpr double expected = -1.26314e+16;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 439") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.65675,
			.beta = 4.92722,
			.a = -2.15126,
			.b = -8.93697e-07,
			.c = -0.612733,
		};
		constexpr double pressure_at_stc = 158.466;
		constexpr double tire_load = 2827.64;
		constexpr double vehicle_speed = 28.705;
		constexpr double expected = -1.16431e+42;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 440") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.60776,
			.beta = 4.31002,
			.a = 9.15881,
			.b = -6.78673e-06,
			.c = -0.92122,
		};
		constexpr double pressure_at_stc = 152.68;
		constexpr double tire_load = 2597.22;
		constexpr double vehicle_speed = 40.9813;
		constexpr double expected = -3.21784e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 441") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.02605,
			.beta = 6.2666,
			.a = -9.33935,
			.b = -5.47772e-07,
			.c = 0.4128,
		};
		constexpr double pressure_at_stc = 122.113;
		constexpr double tire_load = 7901.55;
		constexpr double vehicle_speed = 5.74629;
		constexpr double expected = 2.0447e+41;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 442") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.96754,
			.beta = -5.9958,
			.a = -2.07909,
			.b = 5.1401e-06,
			.c = 0.0418059,
		};
		constexpr double pressure_at_stc = 164.57;
		constexpr double tire_load = 5571.74;
		constexpr double vehicle_speed = 74.0951;
		constexpr double expected = 1.43655e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 443") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.48272,
			.beta = 0.818642,
			.a = -3.73712,
			.b = 6.79446e-06,
			.c = -0.744961,
		};
		constexpr double pressure_at_stc = 138.444;
		constexpr double tire_load = 8052.29;
		constexpr double vehicle_speed = 97.0641;
		constexpr double expected = -7.87822e+19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 444") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.26616,
			.beta = -3.1902,
			.a = -1.38497,
			.b = 6.02329e-06,
			.c = 0.197263,
		};
		constexpr double pressure_at_stc = 152.616;
		constexpr double tire_load = 32.8335;
		constexpr double vehicle_speed = 14.698;
		constexpr double expected = 3.85926e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 445") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.13364,
			.beta = 7.79428,
			.a = 8.92802,
			.b = -9.7786e-06,
			.c = -0.719076,
		};
		constexpr double pressure_at_stc = 149.097;
		constexpr double tire_load = 9576.52;
		constexpr double vehicle_speed = 95.9357;
		constexpr double expected = -4.38699e+53;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 446") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.97028,
			.beta = 5.52664,
			.a = -3.38229,
			.b = 1.81139e-06,
			.c = 0.768655,
		};
		constexpr double pressure_at_stc = 169.703;
		constexpr double tire_load = 1353.36;
		constexpr double vehicle_speed = 75.4958;
		constexpr double expected = 3.30223e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 447") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.14323,
			.beta = 4.94388,
			.a = 2.51391,
			.b = 8.82629e-06,
			.c = 0.909137,
		};
		constexpr double pressure_at_stc = 123.52;
		constexpr double tire_load = 9673.61;
		constexpr double vehicle_speed = 0.715303;
		constexpr double expected = 9.31771e+11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 448") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.89256,
			.beta = -0.0634752,
			.a = 7.06975,
			.b = 6.96792e-07,
			.c = -0.699664,
		};
		constexpr double pressure_at_stc = 170.421;
		constexpr double tire_load = 2800.19;
		constexpr double vehicle_speed = 18.6527;
		constexpr double expected = -4.64244e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 449") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.69067,
			.beta = 0.28322,
			.a = 2.92869,
			.b = 8.50569e-06,
			.c = 0.28776,
		};
		constexpr double pressure_at_stc = 186.872;
		constexpr double tire_load = 6313.63;
		constexpr double vehicle_speed = 64.9592;
		constexpr double expected = 1.5846e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 450") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.77829,
			.beta = -7.33135,
			.a = 4.58033,
			.b = -6.52641e-06,
			.c = -0.543593,
		};
		constexpr double pressure_at_stc = 178.564;
		constexpr double tire_load = 4098.01;
		constexpr double vehicle_speed = 33.8258;
		constexpr double expected = -0.00154163;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 451") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.94385,
			.beta = -9.84164,
			.a = -9.54796,
			.b = 5.41183e-06,
			.c = -0.317342,
		};
		constexpr double pressure_at_stc = 195.625;
		constexpr double tire_load = 1871.04;
		constexpr double vehicle_speed = 94.9197;
		constexpr double expected = -1.09099e-39;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 452") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.6644,
			.beta = 8.32415,
			.a = 0.698126,
			.b = 2.22506e-06,
			.c = 0.334012,
		};
		constexpr double pressure_at_stc = 143.393;
		constexpr double tire_load = 9703.38;
		constexpr double vehicle_speed = 69.9393;
		constexpr double expected = 7.68292e+51;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 453") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 1.96475,
			.beta = 1.44521,
			.a = 7.96826,
			.b = -6.154e-06,
			.c = -0.64855,
		};
		constexpr double pressure_at_stc = 167.316;
		constexpr double tire_load = 5898.09;
		constexpr double vehicle_speed = 18.2126;
		constexpr double expected = -1.82905e+13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 454") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.98425,
			.beta = 5.50576,
			.a = 7.59272,
			.b = -6.18931e-06,
			.c = -0.873119,
		};
		constexpr double pressure_at_stc = 136.569;
		constexpr double tire_load = 9641.45;
		constexpr double vehicle_speed = 83.2681;
		constexpr double expected = -7.57772e+43;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 455") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.74525,
			.beta = 5.39627,
			.a = -5.47017,
			.b = -7.66695e-08,
			.c = -0.27004,
		};
		constexpr double pressure_at_stc = 135.2;
		constexpr double tire_load = 6251.53;
		constexpr double vehicle_speed = 63.6029;
		constexpr double expected = -3.33659e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 456") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.96319,
			.beta = 0.248601,
			.a = 2.13486,
			.b = -9.0164e-06,
			.c = 0.719931,
		};
		constexpr double pressure_at_stc = 174.425;
		constexpr double tire_load = 3970.94;
		constexpr double vehicle_speed = 36.5491;
		constexpr double expected = 4.20012e-09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 457") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -6.57764,
			.beta = 1.58856,
			.a = -4.12814,
			.b = 1.78596e-06,
			.c = 0.180227,
		};
		constexpr double pressure_at_stc = 170.497;
		constexpr double tire_load = 1515.76;
		constexpr double vehicle_speed = 64.5073;
		constexpr double expected = 2.2943e-06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 458") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.14505,
			.beta = -0.308739,
			.a = -2.72346,
			.b = -8.48943e-06,
			.c = 0.462731,
		};
		constexpr double pressure_at_stc = 128.574;
		constexpr double tire_load = 4141.75;
		constexpr double vehicle_speed = 15.2398;
		constexpr double expected = 5.87251e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 459") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.87095,
			.beta = -6.69239,
			.a = 2.36143,
			.b = 8.77475e-06,
			.c = -0.638176,
		};
		constexpr double pressure_at_stc = 191.117;
		constexpr double tire_load = 9662.38;
		constexpr double vehicle_speed = 30.5859;
		constexpr double expected = -5.01356e-46;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 460") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.07752,
			.beta = 2.34693,
			.a = -0.426786,
			.b = 5.5466e-06,
			.c = -0.914922,
		};
		constexpr double pressure_at_stc = 114.099;
		constexpr double tire_load = 4468.11;
		constexpr double vehicle_speed = 7.89613;
		constexpr double expected = -6.67205e+19;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 461") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.50303,
			.beta = 5.21899,
			.a = -4.94286,
			.b = 4.12319e-06,
			.c = 0.943541,
		};
		constexpr double pressure_at_stc = 106.406;
		constexpr double tire_load = 2873.77;
		constexpr double vehicle_speed = 74.4096;
		constexpr double expected = 1.01791e+32;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 462") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.7296,
			.beta = 9.02652,
			.a = -2.91249,
			.b = -6.50424e-06,
			.c = -0.539615,
		};
		constexpr double pressure_at_stc = 102.207;
		constexpr double tire_load = 9559.09;
		constexpr double vehicle_speed = 84.4161;
		constexpr double expected = -1.32268e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 463") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -0.693852,
			.beta = -0.546782,
			.a = -4.67276,
			.b = 3.84152e-06,
			.c = -0.679216,
		};
		constexpr double pressure_at_stc = 125.002;
		constexpr double tire_load = 3514.03;
		constexpr double vehicle_speed = 29.4304;
		constexpr double expected = -3.08134;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 464") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.61574,
			.beta = 2.68071,
			.a = -3.92849,
			.b = 8.5191e-06,
			.c = -0.289546,
		};
		constexpr double pressure_at_stc = 167.502;
		constexpr double tire_load = 1158.29;
		constexpr double vehicle_speed = 76.4709;
		constexpr double expected = -2.35525e+18;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 465") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.6169,
			.beta = 6.78778,
			.a = -0.833436,
			.b = 3.50567e-06,
			.c = 0.465506,
		};
		constexpr double pressure_at_stc = 146.982;
		constexpr double tire_load = 5432.99;
		constexpr double vehicle_speed = 47.4064;
		constexpr double expected = 1.44504e+48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 466") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.79582,
			.beta = 4.04413,
			.a = 6.64853,
			.b = 3.80324e-06,
			.c = -0.014842,
		};
		constexpr double pressure_at_stc = 121.41;
		constexpr double tire_load = 8531;
		constexpr double vehicle_speed = 77.7128;
		constexpr double expected = -1.6486e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 467") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.52332,
			.beta = 7.94463,
			.a = 3.56635,
			.b = -2.16291e-06,
			.c = 0.0350716,
		};
		constexpr double pressure_at_stc = 112.898;
		constexpr double tire_load = 5337.68;
		constexpr double vehicle_speed = 34.0562;
		constexpr double expected = 6.03012e+47;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 468") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.54593,
			.beta = 1.93666,
			.a = 7.78531,
			.b = 6.9285e-07,
			.c = -0.207393,
		};
		constexpr double pressure_at_stc = 165.607;
		constexpr double tire_load = 749.927;
		constexpr double vehicle_speed = 95.3317;
		constexpr double expected = -3.03119e+24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 469") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -5.93576,
			.beta = 4.93603,
			.a = 4.08156,
			.b = 9.92949e-06,
			.c = -0.438116,
		};
		constexpr double pressure_at_stc = 154.809;
		constexpr double tire_load = 4838.2;
		constexpr double vehicle_speed = 83.9462;
		constexpr double expected = -6.19087e+09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 470") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.79424,
			.beta = -9.76049,
			.a = 0.3202,
			.b = 5.88338e-06,
			.c = 0.522172,
		};
		constexpr double pressure_at_stc = 183.496;
		constexpr double tire_load = 2895.1;
		constexpr double vehicle_speed = 39.9618;
		constexpr double expected = 2.30244e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 471") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -3.49198,
			.beta = -5.06525,
			.a = 7.41506,
			.b = -5.78352e-06,
			.c = -0.0903273,
		};
		constexpr double pressure_at_stc = 189.384;
		constexpr double tire_load = 3506.48;
		constexpr double vehicle_speed = 13.5958;
		constexpr double expected = -2.58223e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 472") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -9.15044,
			.beta = 7.8295,
			.a = 9.81387,
			.b = -5.9852e-06,
			.c = -0.237101,
		};
		constexpr double pressure_at_stc = 124.617;
		constexpr double tire_load = 1190.01;
		constexpr double vehicle_speed = 6.24206;
		constexpr double expected = -8.8233e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 473") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.26262,
			.beta = -3.88692,
			.a = -5.49986,
			.b = -5.41197e-06,
			.c = 0.449452,
		};
		constexpr double pressure_at_stc = 130.621;
		constexpr double tire_load = 1524.7;
		constexpr double vehicle_speed = 67.1469;
		constexpr double expected = 1.06346e-17;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 474") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.59588,
			.beta = -8.127,
			.a = 6.86979,
			.b = 4.3986e-06,
			.c = -0.974384,
		};
		constexpr double pressure_at_stc = 184.876;
		constexpr double tire_load = 1917.6;
		constexpr double vehicle_speed = 63.6167;
		constexpr double expected = -0.605569;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 475") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.63685,
			.beta = 2.17904,
			.a = -3.11692,
			.b = 2.7503e-06,
			.c = 0.237716,
		};
		constexpr double pressure_at_stc = 141.758;
		constexpr double tire_load = 1178.36;
		constexpr double vehicle_speed = 28.2637;
		constexpr double expected = 5.70595e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 476") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.46715,
			.beta = 6.43174,
			.a = 6.69831,
			.b = 7.951e-06,
			.c = -0.544204,
		};
		constexpr double pressure_at_stc = 103.073;
		constexpr double tire_load = 8153.51;
		constexpr double vehicle_speed = 75.3732;
		constexpr double expected = -5.48718e+36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 477") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.42494,
			.beta = -9.42972,
			.a = -4.49658,
			.b = -1.01459e-06,
			.c = 0.0830905,
		};
		constexpr double pressure_at_stc = 112.728;
		constexpr double tire_load = 4354.67;
		constexpr double vehicle_speed = 7.81939;
		constexpr double expected = 3.57728e-24;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 478") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.68443,
			.beta = 2.42747,
			.a = -9.36682,
			.b = 4.81685e-06,
			.c = 0.234967,
		};
		constexpr double pressure_at_stc = 123.076;
		constexpr double tire_load = 2918.02;
		constexpr double vehicle_speed = 53.8507;
		constexpr double expected = 1.40671e+22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 479") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.59218,
			.beta = -5.22709,
			.a = 4.44067,
			.b = -7.03516e-06,
			.c = 0.332266,
		};
		constexpr double pressure_at_stc = 168.871;
		constexpr double tire_load = 2338.3;
		constexpr double vehicle_speed = 17.5149;
		constexpr double expected = 7.58446e+06;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 480") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 6.00369,
			.beta = -1.0821,
			.a = -6.56424,
			.b = 3.5079e-06,
			.c = 0.663598,
		};
		constexpr double pressure_at_stc = 151.684;
		constexpr double tire_load = 3026.9;
		constexpr double vehicle_speed = 0.078662;
		constexpr double expected = -1.38216e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 481") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.78048,
			.beta = 0.336591,
			.a = 6.11941,
			.b = -6.40898e-06,
			.c = 0.0906302,
		};
		constexpr double pressure_at_stc = 121.459;
		constexpr double tire_load = 9962.32;
		constexpr double vehicle_speed = 52.8631;
		constexpr double expected = 3.63509e-14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 482") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.2916,
			.beta = -9.87995,
			.a = 2.50105,
			.b = -1.98203e-06,
			.c = 0.0596514,
		};
		constexpr double pressure_at_stc = 194.659;
		constexpr double tire_load = 9483.28;
		constexpr double vehicle_speed = 85.7914;
		constexpr double expected = 3.02959e-55;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 483") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.29874,
			.beta = 3.5791,
			.a = -8.54828,
			.b = 3.71817e-06,
			.c = 0.67106,
		};
		constexpr double pressure_at_stc = 153.922;
		constexpr double tire_load = 7342.76;
		constexpr double vehicle_speed = 36.7476;
		constexpr double expected = 1.16162e+15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 484") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.15565,
			.beta = -7.99763,
			.a = -7.43514,
			.b = 5.14124e-06,
			.c = -0.125224,
		};
		constexpr double pressure_at_stc = 100.765;
		constexpr double tire_load = 5399.97;
		constexpr double vehicle_speed = 9.38603;
		constexpr double expected = -4.45382e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 485") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.64345,
			.beta = 4.26348,
			.a = -8.69324,
			.b = -9.40478e-06,
			.c = -0.298252,
		};
		constexpr double pressure_at_stc = 128.034;
		constexpr double tire_load = 5256.25;
		constexpr double vehicle_speed = 31.8847;
		constexpr double expected = -2244.36;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 486") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 2.23401,
			.beta = 0.856131,
			.a = -7.69512,
			.b = 6.61225e-06,
			.c = 0.796995,
		};
		constexpr double pressure_at_stc = 133.642;
		constexpr double tire_load = 6445.65;
		constexpr double vehicle_speed = 54.6329;
		constexpr double expected = 3.15837e+12;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 487") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 7.59498,
			.beta = 9.5965,
			.a = -5.29091,
			.b = -4.45942e-06,
			.c = -0.870447,
		};
		constexpr double pressure_at_stc = 106.639;
		constexpr double tire_load = 6596.93;
		constexpr double vehicle_speed = 45.6952;
		constexpr double expected = -2.66972e+56;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 488") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 0.166914,
			.beta = 1.61706,
			.a = 2.3075,
			.b = -5.61936e-06,
			.c = -0.730236,
		};
		constexpr double pressure_at_stc = 177.293;
		constexpr double tire_load = 2560.74;
		constexpr double vehicle_speed = 98.0946;
		constexpr double expected = -7.01832e+10;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 489") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -4.10948,
			.beta = 4.76588,
			.a = -0.440382,
			.b = 9.30104e-06,
			.c = -0.306205,
		};
		constexpr double pressure_at_stc = 153.92;
		constexpr double tire_load = 9886.15;
		constexpr double vehicle_speed = 32.9737;
		constexpr double expected = -4.85462e+13;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 490") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 9.35933,
			.beta = -8.59125,
			.a = -0.368347,
			.b = -2.39854e-06,
			.c = 0.926031,
		};
		constexpr double pressure_at_stc = 103.19;
		constexpr double tire_load = 7830.42;
		constexpr double vehicle_speed = 38.6431;
		constexpr double expected = 4.43759e-11;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 491") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -2.84282,
			.beta = -3.58982,
			.a = 1.13202,
			.b = 7.50804e-06,
			.c = -0.579291,
		};
		constexpr double pressure_at_stc = 198.997;
		constexpr double tire_load = 4401.59;
		constexpr double vehicle_speed = 83.6797;
		constexpr double expected = -1.27534e-15;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 492") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 8.68041,
			.beta = -2.46014,
			.a = 1.33923,
			.b = -9.4738e-06,
			.c = 0.91432,
		};
		constexpr double pressure_at_stc = 149.369;
		constexpr double tire_load = 6561.35;
		constexpr double vehicle_speed = 90.5543;
		constexpr double expected = 2.95591e+14;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 493") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -8.92785,
			.beta = -9.64165,
			.a = 8.31089,
			.b = -2.06906e-06,
			.c = 0.367509,
		};
		constexpr double pressure_at_stc = 137.909;
		constexpr double tire_load = 1961.95;
		constexpr double vehicle_speed = 38.1385;
		constexpr double expected = 9.82248e-48;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 494") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.38988,
			.beta = -1.79776,
			.a = 6.11804,
			.b = 3.79877e-06,
			.c = 0.0227607,
		};
		constexpr double pressure_at_stc = 129.101;
		constexpr double tire_load = 8357.51;
		constexpr double vehicle_speed = 9.63241;
		constexpr double expected = 7.48843e-22;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinAbs(expected, EPSILON));
	}
	SECTION("Random Test 495") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -1.69977,
			.beta = 1.05216,
			.a = 6.81657,
			.b = 6.88026e-06,
			.c = 0.650766,
		};
		constexpr double pressure_at_stc = 113.948;
		constexpr double tire_load = 1668.07;
		constexpr double vehicle_speed = 87.8086;
		constexpr double expected = 50993.8;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 496") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = -7.0032,
			.beta = 9.20215,
			.a = 5.20309,
			.b = 1.32597e-06,
			.c = -0.666237,
		};
		constexpr double pressure_at_stc = 156.265;
		constexpr double tire_load = 4923.67;
		constexpr double vehicle_speed = 65.9433;
		constexpr double expected = -1.53977e+23;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 497") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 4.40496,
			.beta = 6.77317,
			.a = 1.6456,
			.b = -1.1655e-06,
			.c = 0.934715,
		};
		constexpr double pressure_at_stc = 141.822;
		constexpr double tire_load = 9602.97;
		constexpr double vehicle_speed = 24.3004;
		constexpr double expected = 2.02524e+40;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 498") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 5.44429,
			.beta = -2.04543,
			.a = -4.671,
			.b = 7.66652e-07,
			.c = 0.719879,
		};
		constexpr double pressure_at_stc = 162.94;
		constexpr double tire_load = 9155.24;
		constexpr double vehicle_speed = 94.4139;
		constexpr double expected = 7.23657e+08;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 499") {
		constexpr auto coefficients = SaeJ2452Coefficients {
			.alpha = 3.62233,
			.beta = -0.273755,
			.a = -4.83027,
			.b = -1.26489e-06,
			.c = 0.611307,
		};
		constexpr double pressure_at_stc = 108.445;
		constexpr double tire_load = 1639.47;
		constexpr double vehicle_speed = 19.8427;
		constexpr double expected = 9.67274e+09;
		const auto tire = Tire(coefficients, pressure_at_stc);
		const auto result = tire.rolling_resistance(tire_load, vehicle_speed);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
}
