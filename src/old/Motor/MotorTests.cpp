#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <numbers>

#include "Motor.h"

constexpr double EPSILON = 0.001; // %

using Catch::Matchers::WithinRel;
using Catch::Matchers::WithinAbs;

TEST_CASE("Motor: power_consumed", "[Motor]") {
	SECTION("Random Test 0") {
		constexpr double hysteresis_loss = 3.15188;
		constexpr double eddy_current_loss_coefficient = 0.0350765;
		constexpr double angular_speed = -24.4341;
		constexpr double torque = 39.6125;
		constexpr double expected = -965.601;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 1") {
		constexpr double hysteresis_loss = 4.23499;
		constexpr double eddy_current_loss_coefficient = 0.00308521;
		constexpr double angular_speed = -4.33354;
		constexpr double torque = 59.3109;
		constexpr double expected = -252.804;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 2") {
		constexpr double hysteresis_loss = 1.23401;
		constexpr double eddy_current_loss_coefficient = 0.0195381;
		constexpr double angular_speed = 2.84317;
		constexpr double torque = 47.1822;
		constexpr double expected = 135.436;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 3") {
		constexpr double hysteresis_loss = 3.87165;
		constexpr double eddy_current_loss_coefficient = 0.0105978;
		constexpr double angular_speed = -26.9019;
		constexpr double torque = 42.5219;
		constexpr double expected = -1140.33;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 4") {
		constexpr double hysteresis_loss = 3.17084;
		constexpr double eddy_current_loss_coefficient = 0.0212486;
		constexpr double angular_speed = -16.4961;
		constexpr double torque = 67.5397;
		constexpr double expected = -1111.32;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 5") {
		constexpr double hysteresis_loss = 2.09383;
		constexpr double eddy_current_loss_coefficient = 0.0404862;
		constexpr double angular_speed = -12.1912;
		constexpr double torque = 72.3726;
		constexpr double expected = -880.709;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 6") {
		constexpr double hysteresis_loss = 1.54949;
		constexpr double eddy_current_loss_coefficient = 0.0154736;
		constexpr double angular_speed = 17.3288;
		constexpr double torque = 93.4134;
		constexpr double expected = 1620.56;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 7") {
		constexpr double hysteresis_loss = 1.03122;
		constexpr double eddy_current_loss_coefficient = 0.0419313;
		constexpr double angular_speed = 16.7879;
		constexpr double torque = 96.9808;
		constexpr double expected = 1629.84;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 8") {
		constexpr double hysteresis_loss = 4.10305;
		constexpr double eddy_current_loss_coefficient = 0.0353373;
		constexpr double angular_speed = 0.279934;
		constexpr double torque = 93.2719;
		constexpr double expected = 30.223;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 9") {
		constexpr double hysteresis_loss = 4.39014;
		constexpr double eddy_current_loss_coefficient = 0.027991;
		constexpr double angular_speed = -16.7895;
		constexpr double torque = 21.633;
		constexpr double expected = -359.288;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 10") {
		constexpr double hysteresis_loss = 2.00919;
		constexpr double eddy_current_loss_coefficient = 0.042633;
		constexpr double angular_speed = -22.2778;
		constexpr double torque = 30.1138;
		constexpr double expected = -669.81;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 11") {
		constexpr double hysteresis_loss = 4.49028;
		constexpr double eddy_current_loss_coefficient = 0.00271003;
		constexpr double angular_speed = 26.2372;
		constexpr double torque = 100.31;
		constexpr double expected = 2636.41;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 12") {
		constexpr double hysteresis_loss = 3.14936;
		constexpr double eddy_current_loss_coefficient = 0.0161505;
		constexpr double angular_speed = 18.9922;
		constexpr double torque = 6.34123;
		constexpr double expected = 123.89;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 13") {
		constexpr double hysteresis_loss = 3.16878;
		constexpr double eddy_current_loss_coefficient = 0.0358914;
		constexpr double angular_speed = 18.2556;
		constexpr double torque = 92.5023;
		constexpr double expected = 1692.51;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 14") {
		constexpr double hysteresis_loss = 3.07349;
		constexpr double eddy_current_loss_coefficient = 0.0176112;
		constexpr double angular_speed = -15.7619;
		constexpr double torque = 71.1797;
		constexpr double expected = -1119.14;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 15") {
		constexpr double hysteresis_loss = 4.45632;
		constexpr double eddy_current_loss_coefficient = 0.00713995;
		constexpr double angular_speed = -18.7443;
		constexpr double torque = 66.851;
		constexpr double expected = -1248.75;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 16") {
		constexpr double hysteresis_loss = 4.42339;
		constexpr double eddy_current_loss_coefficient = 0.00250313;
		constexpr double angular_speed = 5.18685;
		constexpr double torque = 110.408;
		constexpr double expected = 577.108;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 17") {
		constexpr double hysteresis_loss = 1.60257;
		constexpr double eddy_current_loss_coefficient = 0.028092;
		constexpr double angular_speed = -23.9741;
		constexpr double torque = 78.4265;
		constexpr double expected = -1879.27;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 18") {
		constexpr double hysteresis_loss = 1.83755;
		constexpr double eddy_current_loss_coefficient = 0.0430191;
		constexpr double angular_speed = -20.6076;
		constexpr double torque = 85.0978;
		constexpr double expected = -1752.71;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 19") {
		constexpr double hysteresis_loss = 3.7258;
		constexpr double eddy_current_loss_coefficient = 0.018528;
		constexpr double angular_speed = -1.33556;
		constexpr double torque = 29.7794;
		constexpr double expected = -36.0711;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 20") {
		constexpr double hysteresis_loss = 2.85062;
		constexpr double eddy_current_loss_coefficient = 0.00605976;
		constexpr double angular_speed = 7.20758;
		constexpr double torque = 83.8434;
		constexpr double expected = 607.202;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 21") {
		constexpr double hysteresis_loss = 2.64661;
		constexpr double eddy_current_loss_coefficient = 0.0115422;
		constexpr double angular_speed = 16.6704;
		constexpr double torque = 106.93;
		constexpr double expected = 1785.41;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 22") {
		constexpr double hysteresis_loss = 2.64434;
		constexpr double eddy_current_loss_coefficient = 0.0428573;
		constexpr double angular_speed = 5.20027;
		constexpr double torque = 24.8031;
		constexpr double expected = 131.85;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 23") {
		constexpr double hysteresis_loss = 2.85269;
		constexpr double eddy_current_loss_coefficient = 0.0287162;
		constexpr double angular_speed = 18.4307;
		constexpr double torque = 23.8824;
		constexpr double expected = 443.552;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 24") {
		constexpr double hysteresis_loss = 1.91689;
		constexpr double eddy_current_loss_coefficient = 0.0165157;
		constexpr double angular_speed = -22.5413;
		constexpr double torque = 55.9545;
		constexpr double expected = -1259.74;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 25") {
		constexpr double hysteresis_loss = 4.40043;
		constexpr double eddy_current_loss_coefficient = 0.013843;
		constexpr double angular_speed = -19.3706;
		constexpr double torque = 27.114;
		constexpr double expected = -521.081;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 26") {
		constexpr double hysteresis_loss = 4.09212;
		constexpr double eddy_current_loss_coefficient = 0.038797;
		constexpr double angular_speed = -13.0817;
		constexpr double torque = 20.1727;
		constexpr double expected = -260.309;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 27") {
		constexpr double hysteresis_loss = 1.39554;
		constexpr double eddy_current_loss_coefficient = 0.0423318;
		constexpr double angular_speed = -8.53799;
		constexpr double torque = 115.581;
		constexpr double expected = -985.796;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 28") {
		constexpr double hysteresis_loss = 3.70804;
		constexpr double eddy_current_loss_coefficient = 0.0054466;
		constexpr double angular_speed = 7.12202;
		constexpr double torque = 15.787;
		constexpr double expected = 116.182;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 29") {
		constexpr double hysteresis_loss = 3.78922;
		constexpr double eddy_current_loss_coefficient = 0.00353428;
		constexpr double angular_speed = 7.74874;
		constexpr double torque = 108.248;
		constexpr double expected = 842.6;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 30") {
		constexpr double hysteresis_loss = 3.10683;
		constexpr double eddy_current_loss_coefficient = 0.0123365;
		constexpr double angular_speed = -29.316;
		constexpr double torque = 104.19;
		constexpr double expected = -3051.68;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 31") {
		constexpr double hysteresis_loss = 1.44486;
		constexpr double eddy_current_loss_coefficient = 0.0109528;
		constexpr double angular_speed = 10.7583;
		constexpr double torque = 95.9212;
		constexpr double expected = 1033.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 32") {
		constexpr double hysteresis_loss = 4.46868;
		constexpr double eddy_current_loss_coefficient = 0.00289578;
		constexpr double angular_speed = -7.00461;
		constexpr double torque = 46.0391;
		constexpr double expected = -318.037;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 33") {
		constexpr double hysteresis_loss = 1.97208;
		constexpr double eddy_current_loss_coefficient = 0.044773;
		constexpr double angular_speed = 9.27019;
		constexpr double torque = 112.514;
		constexpr double expected = 1045.41;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 34") {
		constexpr double hysteresis_loss = 2.61261;
		constexpr double eddy_current_loss_coefficient = 0.00179425;
		constexpr double angular_speed = 16.4095;
		constexpr double torque = 62.0825;
		constexpr double expected = 1021.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 35") {
		constexpr double hysteresis_loss = 1.4693;
		constexpr double eddy_current_loss_coefficient = 0.0423217;
		constexpr double angular_speed = -17.5864;
		constexpr double torque = 41.4654;
		constexpr double expected = -728.503;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 36") {
		constexpr double hysteresis_loss = 4.86245;
		constexpr double eddy_current_loss_coefficient = 0.00719263;
		constexpr double angular_speed = -15.6807;
		constexpr double torque = 45.5247;
		constexpr double expected = -709.111;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 37") {
		constexpr double hysteresis_loss = 3.50752;
		constexpr double eddy_current_loss_coefficient = 0.0417879;
		constexpr double angular_speed = 9.56239;
		constexpr double torque = 11.5446;
		constexpr double expected = 114.301;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 38") {
		constexpr double hysteresis_loss = 1.65;
		constexpr double eddy_current_loss_coefficient = 0.0474606;
		constexpr double angular_speed = 20.3366;
		constexpr double torque = 10.9858;
		constexpr double expected = 226.029;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 39") {
		constexpr double hysteresis_loss = 1.10589;
		constexpr double eddy_current_loss_coefficient = 0.0461396;
		constexpr double angular_speed = -10.0172;
		constexpr double torque = 9.30397;
		constexpr double expected = -92.5556;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 40") {
		constexpr double hysteresis_loss = 3.53757;
		constexpr double eddy_current_loss_coefficient = 0.0163162;
		constexpr double angular_speed = -25.0357;
		constexpr double torque = 43.5176;
		constexpr double expected = -1086.36;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 41") {
		constexpr double hysteresis_loss = 1.08938;
		constexpr double eddy_current_loss_coefficient = 0.00869668;
		constexpr double angular_speed = -5.1245;
		constexpr double torque = 122.524;
		constexpr double expected = -626.828;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 42") {
		constexpr double hysteresis_loss = 3.26874;
		constexpr double eddy_current_loss_coefficient = 0.00383611;
		constexpr double angular_speed = 24.6321;
		constexpr double torque = 118.219;
		constexpr double expected = 2915.33;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 43") {
		constexpr double hysteresis_loss = 1.01701;
		constexpr double eddy_current_loss_coefficient = 0.00417348;
		constexpr double angular_speed = 22.4734;
		constexpr double torque = 80.8844;
		constexpr double expected = 1818.86;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 44") {
		constexpr double hysteresis_loss = 4.24662;
		constexpr double eddy_current_loss_coefficient = 0.0315694;
		constexpr double angular_speed = 27.4656;
		constexpr double torque = 45.3287;
		constexpr double expected = 1250.1;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 45") {
		constexpr double hysteresis_loss = 1.5276;
		constexpr double eddy_current_loss_coefficient = 0.0157908;
		constexpr double angular_speed = -20.3168;
		constexpr double torque = 115.217;
		constexpr double expected = -2339.63;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 46") {
		constexpr double hysteresis_loss = 2.72107;
		constexpr double eddy_current_loss_coefficient = 0.0377692;
		constexpr double angular_speed = -29.8142;
		constexpr double torque = 122.863;
		constexpr double expected = -3661.46;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 47") {
		constexpr double hysteresis_loss = 4.86229;
		constexpr double eddy_current_loss_coefficient = 0.0188815;
		constexpr double angular_speed = -17.0101;
		constexpr double torque = 38.0995;
		constexpr double expected = -643.537;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 48") {
		constexpr double hysteresis_loss = 2.4185;
		constexpr double eddy_current_loss_coefficient = 0.0339405;
		constexpr double angular_speed = 1.29659;
		constexpr double torque = 5.6773;
		constexpr double expected = 9.82362;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 49") {
		constexpr double hysteresis_loss = 1.2239;
		constexpr double eddy_current_loss_coefficient = 0.0482715;
		constexpr double angular_speed = 5.41057;
		constexpr double torque = 26.6807;
		constexpr double expected = 145.843;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 50") {
		constexpr double hysteresis_loss = 1.7204;
		constexpr double eddy_current_loss_coefficient = 0.00420385;
		constexpr double angular_speed = -0.411316;
		constexpr double torque = 55.1673;
		constexpr double expected = -20.9725;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 51") {
		constexpr double hysteresis_loss = 3.90997;
		constexpr double eddy_current_loss_coefficient = 0.0323316;
		constexpr double angular_speed = 6.46404;
		constexpr double torque = 123.502;
		constexpr double expected = 802.441;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 52") {
		constexpr double hysteresis_loss = 4.61042;
		constexpr double eddy_current_loss_coefficient = 0.0193914;
		constexpr double angular_speed = -22.4291;
		constexpr double torque = 23.3989;
		constexpr double expected = -520.639;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 53") {
		constexpr double hysteresis_loss = 2.68406;
		constexpr double eddy_current_loss_coefficient = 0.0442521;
		constexpr double angular_speed = -12.4435;
		constexpr double torque = 115.593;
		constexpr double expected = -1436.26;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 54") {
		constexpr double hysteresis_loss = 4.77499;
		constexpr double eddy_current_loss_coefficient = 0.0187549;
		constexpr double angular_speed = 8.85967;
		constexpr double torque = 123.357;
		constexpr double expected = 1097.84;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 55") {
		constexpr double hysteresis_loss = 4.02759;
		constexpr double eddy_current_loss_coefficient = 0.0439447;
		constexpr double angular_speed = 19.5663;
		constexpr double torque = 6.11604;
		constexpr double expected = 124.556;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 56") {
		constexpr double hysteresis_loss = 1.38208;
		constexpr double eddy_current_loss_coefficient = 0.0158407;
		constexpr double angular_speed = 17.1324;
		constexpr double torque = 96.9564;
		constexpr double expected = 1662.75;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 57") {
		constexpr double hysteresis_loss = 1.00503;
		constexpr double eddy_current_loss_coefficient = 0.00469066;
		constexpr double angular_speed = -9.84115;
		constexpr double torque = 1.69584;
		constexpr double expected = -15.7301;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 58") {
		constexpr double hysteresis_loss = 4.18509;
		constexpr double eddy_current_loss_coefficient = 0.039489;
		constexpr double angular_speed = 3.72477;
		constexpr double torque = 112.234;
		constexpr double expected = 422.378;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 59") {
		constexpr double hysteresis_loss = 1.63523;
		constexpr double eddy_current_loss_coefficient = 0.00399889;
		constexpr double angular_speed = 0.100591;
		constexpr double torque = 59.6043;
		constexpr double expected = 7.63132;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 60") {
		constexpr double hysteresis_loss = 2.00554;
		constexpr double eddy_current_loss_coefficient = 0.0257266;
		constexpr double angular_speed = 17.7063;
		constexpr double torque = 24.6758;
		constexpr double expected = 439.379;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 61") {
		constexpr double hysteresis_loss = 3.99247;
		constexpr double eddy_current_loss_coefficient = 0.0253003;
		constexpr double angular_speed = -11.8207;
		constexpr double torque = 79.9929;
		constexpr double expected = -941.88;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 62") {
		constexpr double hysteresis_loss = 3.23935;
		constexpr double eddy_current_loss_coefficient = 0.00348935;
		constexpr double angular_speed = -9.95843;
		constexpr double torque = 52.3292;
		constexpr double expected = -517.912;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 63") {
		constexpr double hysteresis_loss = 2.80419;
		constexpr double eddy_current_loss_coefficient = 0.0189293;
		constexpr double angular_speed = 4.39595;
		constexpr double torque = 14.3022;
		constexpr double expected = 65.7593;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 64") {
		constexpr double hysteresis_loss = 4.08679;
		constexpr double eddy_current_loss_coefficient = 0.00659942;
		constexpr double angular_speed = -7.57965;
		constexpr double torque = 94.1445;
		constexpr double expected = -709.546;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 65") {
		constexpr double hysteresis_loss = 1.09145;
		constexpr double eddy_current_loss_coefficient = 0.0281286;
		constexpr double angular_speed = -17.3628;
		constexpr double torque = 89.0144;
		constexpr double expected = -1544.94;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 66") {
		constexpr double hysteresis_loss = 3.25698;
		constexpr double eddy_current_loss_coefficient = 0.00837205;
		constexpr double angular_speed = 24.5892;
		constexpr double torque = 91.9367;
		constexpr double expected = 2264.11;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 67") {
		constexpr double hysteresis_loss = 3.85129;
		constexpr double eddy_current_loss_coefficient = 0.0231644;
		constexpr double angular_speed = 22.84;
		constexpr double torque = 75.6147;
		constexpr double expected = 1731.42;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 68") {
		constexpr double hysteresis_loss = 4.66709;
		constexpr double eddy_current_loss_coefficient = 0.0433709;
		constexpr double angular_speed = -10.9265;
		constexpr double torque = 60.8105;
		constexpr double expected = -660.251;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 69") {
		constexpr double hysteresis_loss = 2.52217;
		constexpr double eddy_current_loss_coefficient = 0.0281933;
		constexpr double angular_speed = 28.4887;
		constexpr double torque = 62.5191;
		constexpr double expected = 1784.41;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 70") {
		constexpr double hysteresis_loss = 3.36735;
		constexpr double eddy_current_loss_coefficient = 0.0118402;
		constexpr double angular_speed = 23.5269;
		constexpr double torque = 55.5665;
		constexpr double expected = 1310.95;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 71") {
		constexpr double hysteresis_loss = 2.24323;
		constexpr double eddy_current_loss_coefficient = 0.0321683;
		constexpr double angular_speed = -2.97166;
		constexpr double torque = 57.3787;
		constexpr double expected = -168.363;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 72") {
		constexpr double hysteresis_loss = 3.32462;
		constexpr double eddy_current_loss_coefficient = 0.0178905;
		constexpr double angular_speed = 24.1322;
		constexpr double torque = 54.9438;
		constexpr double expected = 1329.67;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 73") {
		constexpr double hysteresis_loss = 3.07859;
		constexpr double eddy_current_loss_coefficient = 0.0388881;
		constexpr double angular_speed = -14.7104;
		constexpr double torque = 96.0656;
		constexpr double expected = -1410.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 74") {
		constexpr double hysteresis_loss = 4.27623;
		constexpr double eddy_current_loss_coefficient = 0.0497482;
		constexpr double angular_speed = 5.44496;
		constexpr double torque = 53.7439;
		constexpr double expected = 297.181;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 75") {
		constexpr double hysteresis_loss = 3.09239;
		constexpr double eddy_current_loss_coefficient = 0.00973199;
		constexpr double angular_speed = -27.0058;
		constexpr double torque = 73.3584;
		constexpr double expected = -1978.27;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 76") {
		constexpr double hysteresis_loss = 3.91417;
		constexpr double eddy_current_loss_coefficient = 0.0222423;
		constexpr double angular_speed = -6.48001;
		constexpr double torque = 80.768;
		constexpr double expected = -519.607;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 77") {
		constexpr double hysteresis_loss = 1.55713;
		constexpr double eddy_current_loss_coefficient = 0.0259996;
		constexpr double angular_speed = 22.8536;
		constexpr double torque = 85.9618;
		constexpr double expected = 1966.69;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 78") {
		constexpr double hysteresis_loss = 4.65246;
		constexpr double eddy_current_loss_coefficient = 0.045407;
		constexpr double angular_speed = 13.6221;
		constexpr double torque = 9.40044;
		constexpr double expected = 133.325;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 79") {
		constexpr double hysteresis_loss = 2.83682;
		constexpr double eddy_current_loss_coefficient = 0.048546;
		constexpr double angular_speed = -23.9991;
		constexpr double torque = 82.7774;
		constexpr double expected = -1984.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 80") {
		constexpr double hysteresis_loss = 4.63991;
		constexpr double eddy_current_loss_coefficient = 0.0285021;
		constexpr double angular_speed = 9.52234;
		constexpr double torque = 53.4049;
		constexpr double expected = 513.451;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 81") {
		constexpr double hysteresis_loss = 2.96683;
		constexpr double eddy_current_loss_coefficient = 0.00762459;
		constexpr double angular_speed = -27.5967;
		constexpr double torque = 67.082;
		constexpr double expected = -1848.48;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 82") {
		constexpr double hysteresis_loss = 2.92182;
		constexpr double eddy_current_loss_coefficient = 0.0210497;
		constexpr double angular_speed = -28.9865;
		constexpr double torque = 116.887;
		constexpr double expected = -3385.82;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 83") {
		constexpr double hysteresis_loss = 1.09415;
		constexpr double eddy_current_loss_coefficient = 0.0330096;
		constexpr double angular_speed = 11.8231;
		constexpr double torque = 104.847;
		constexpr double expected = 1241.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 84") {
		constexpr double hysteresis_loss = 4.49147;
		constexpr double eddy_current_loss_coefficient = 0.0119261;
		constexpr double angular_speed = 28.8544;
		constexpr double torque = 68.7532;
		constexpr double expected = 1988.67;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 85") {
		constexpr double hysteresis_loss = 4.26784;
		constexpr double eddy_current_loss_coefficient = 0.00537922;
		constexpr double angular_speed = 29.2265;
		constexpr double torque = 118.171;
		constexpr double expected = 3458.15;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 86") {
		constexpr double hysteresis_loss = 3.81591;
		constexpr double eddy_current_loss_coefficient = 0.0307335;
		constexpr double angular_speed = 18.627;
		constexpr double torque = 115.673;
		constexpr double expected = 2159.02;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 87") {
		constexpr double hysteresis_loss = 4.87762;
		constexpr double eddy_current_loss_coefficient = 0.0492026;
		constexpr double angular_speed = -21.3836;
		constexpr double torque = 34.5033;
		constexpr double expected = -733.98;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 88") {
		constexpr double hysteresis_loss = 1.66524;
		constexpr double eddy_current_loss_coefficient = 0.0216872;
		constexpr double angular_speed = 6.84049;
		constexpr double torque = 100.018;
		constexpr double expected = 685.983;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 89") {
		constexpr double hysteresis_loss = 3.10113;
		constexpr double eddy_current_loss_coefficient = 0.0267965;
		constexpr double angular_speed = -22.551;
		constexpr double torque = 48.6239;
		constexpr double expected = -1094.02;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 90") {
		constexpr double hysteresis_loss = 1.14778;
		constexpr double eddy_current_loss_coefficient = 0.0446483;
		constexpr double angular_speed = 26.8393;
		constexpr double torque = 22.4965;
		constexpr double expected = 606.136;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 91") {
		constexpr double hysteresis_loss = 1.84866;
		constexpr double eddy_current_loss_coefficient = 0.0160288;
		constexpr double angular_speed = 2.04279;
		constexpr double torque = 80.9815;
		constexpr double expected = 167.31;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 92") {
		constexpr double hysteresis_loss = 1.06613;
		constexpr double eddy_current_loss_coefficient = 0.0109438;
		constexpr double angular_speed = 29.0879;
		constexpr double torque = 22.0028;
		constexpr double expected = 641.401;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 93") {
		constexpr double hysteresis_loss = 2.35369;
		constexpr double eddy_current_loss_coefficient = 0.0287909;
		constexpr double angular_speed = 18.4456;
		constexpr double torque = 65.7347;
		constexpr double expected = 1215.4;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 94") {
		constexpr double hysteresis_loss = 1.88828;
		constexpr double eddy_current_loss_coefficient = 0.0355229;
		constexpr double angular_speed = 19.4148;
		constexpr double torque = 14.6431;
		constexpr double expected = 286.871;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 95") {
		constexpr double hysteresis_loss = 3.34117;
		constexpr double eddy_current_loss_coefficient = 0.0439234;
		constexpr double angular_speed = 5.8402;
		constexpr double torque = 104.81;
		constexpr double expected = 615.71;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 96") {
		constexpr double hysteresis_loss = 3.79251;
		constexpr double eddy_current_loss_coefficient = 0.0338447;
		constexpr double angular_speed = 9.9729;
		constexpr double torque = 101.058;
		constexpr double expected = 1011.97;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 97") {
		constexpr double hysteresis_loss = 2.46624;
		constexpr double eddy_current_loss_coefficient = 0.013682;
		constexpr double angular_speed = 13.6876;
		constexpr double torque = 44.3482;
		constexpr double expected = 609.676;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 98") {
		constexpr double hysteresis_loss = 2.7642;
		constexpr double eddy_current_loss_coefficient = 0.0111323;
		constexpr double angular_speed = -8.01911;
		constexpr double torque = 113.994;
		constexpr double expected = -911.453;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 99") {
		constexpr double hysteresis_loss = 3.71504;
		constexpr double eddy_current_loss_coefficient = 0.04147;
		constexpr double angular_speed = -18.7017;
		constexpr double torque = 32.6611;
		constexpr double expected = -607.878;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 100") {
		constexpr double hysteresis_loss = 4.85323;
		constexpr double eddy_current_loss_coefficient = 0.0494354;
		constexpr double angular_speed = 12.6615;
		constexpr double torque = 5.56554;
		constexpr double expected = 75.9474;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 101") {
		constexpr double hysteresis_loss = 4.69174;
		constexpr double eddy_current_loss_coefficient = 0.0288908;
		constexpr double angular_speed = 22.4171;
		constexpr double torque = 118.774;
		constexpr double expected = 2667.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 102") {
		constexpr double hysteresis_loss = 1.03217;
		constexpr double eddy_current_loss_coefficient = 0.0100744;
		constexpr double angular_speed = 19.7973;
		constexpr double torque = 71.6599;
		constexpr double expected = 1419.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 103") {
		constexpr double hysteresis_loss = 1.77223;
		constexpr double eddy_current_loss_coefficient = 0.0335223;
		constexpr double angular_speed = -29.9163;
		constexpr double torque = 28.3912;
		constexpr double expected = -848.591;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 104") {
		constexpr double hysteresis_loss = 2.13677;
		constexpr double eddy_current_loss_coefficient = 0.0298421;
		constexpr double angular_speed = 14.0201;
		constexpr double torque = 103.144;
		constexpr double expected = 1448.65;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 105") {
		constexpr double hysteresis_loss = 2.69511;
		constexpr double eddy_current_loss_coefficient = 0.0496852;
		constexpr double angular_speed = 16.8326;
		constexpr double torque = 92.5831;
		constexpr double expected = 1561.94;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 106") {
		constexpr double hysteresis_loss = 4.38092;
		constexpr double eddy_current_loss_coefficient = 0.0358491;
		constexpr double angular_speed = -16.0936;
		constexpr double torque = 43.6606;
		constexpr double expected = -698.851;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 107") {
		constexpr double hysteresis_loss = 3.19326;
		constexpr double eddy_current_loss_coefficient = 0.0210946;
		constexpr double angular_speed = -12.87;
		constexpr double torque = 84.6688;
		constexpr double expected = -1086.76;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 108") {
		constexpr double hysteresis_loss = 4.66651;
		constexpr double eddy_current_loss_coefficient = 0.0321605;
		constexpr double angular_speed = 17.0425;
		constexpr double torque = 60.9741;
		constexpr double expected = 1044.36;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 109") {
		constexpr double hysteresis_loss = 4.57131;
		constexpr double eddy_current_loss_coefficient = 0.0249232;
		constexpr double angular_speed = 27.2642;
		constexpr double torque = 22.3346;
		constexpr double expected = 614.186;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 110") {
		constexpr double hysteresis_loss = 3.21638;
		constexpr double eddy_current_loss_coefficient = 0.0400191;
		constexpr double angular_speed = -21.3293;
		constexpr double torque = 116.056;
		constexpr double expected = -2473.04;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 111") {
		constexpr double hysteresis_loss = 2.45451;
		constexpr double eddy_current_loss_coefficient = 0.0395319;
		constexpr double angular_speed = 22.5584;
		constexpr double torque = 50.4794;
		constexpr double expected = 1142.08;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 112") {
		constexpr double hysteresis_loss = 3.17533;
		constexpr double eddy_current_loss_coefficient = 0.00368977;
		constexpr double angular_speed = -15.0798;
		constexpr double torque = 6.0211;
		constexpr double expected = -87.6771;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 113") {
		constexpr double hysteresis_loss = 4.31046;
		constexpr double eddy_current_loss_coefficient = 0.0185241;
		constexpr double angular_speed = 2.1181;
		constexpr double torque = 71.9033;
		constexpr double expected = 156.648;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 114") {
		constexpr double hysteresis_loss = 3.2198;
		constexpr double eddy_current_loss_coefficient = 0.0491871;
		constexpr double angular_speed = 12.6487;
		constexpr double torque = 63.4521;
		constexpr double expected = 806.427;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 115") {
		constexpr double hysteresis_loss = 3.85913;
		constexpr double eddy_current_loss_coefficient = 0.0205744;
		constexpr double angular_speed = 18.7355;
		constexpr double torque = 49.8901;
		constexpr double expected = 938.958;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 116") {
		constexpr double hysteresis_loss = 2.0196;
		constexpr double eddy_current_loss_coefficient = 0.0059433;
		constexpr double angular_speed = -5.01738;
		constexpr double torque = 92.0941;
		constexpr double expected = -460.082;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 117") {
		constexpr double hysteresis_loss = 2.58736;
		constexpr double eddy_current_loss_coefficient = 0.0431709;
		constexpr double angular_speed = -22.0781;
		constexpr double torque = 9.10293;
		constexpr double expected = -199.341;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 118") {
		constexpr double hysteresis_loss = 4.73179;
		constexpr double eddy_current_loss_coefficient = 0.00728514;
		constexpr double angular_speed = -24.424;
		constexpr double torque = 75.2832;
		constexpr double expected = -1834.17;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 119") {
		constexpr double hysteresis_loss = 3.90796;
		constexpr double eddy_current_loss_coefficient = 0.0214098;
		constexpr double angular_speed = 26.7913;
		constexpr double torque = 10.1229;
		constexpr double expected = 275.687;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 120") {
		constexpr double hysteresis_loss = 1.67815;
		constexpr double eddy_current_loss_coefficient = 0.0112752;
		constexpr double angular_speed = -1.21981;
		constexpr double torque = 35.7859;
		constexpr double expected = -41.9876;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 121") {
		constexpr double hysteresis_loss = 3.98089;
		constexpr double eddy_current_loss_coefficient = 0.0320253;
		constexpr double angular_speed = 20.0809;
		constexpr double torque = 86.4943;
		constexpr double expected = 1741.51;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 122") {
		constexpr double hysteresis_loss = 3.36369;
		constexpr double eddy_current_loss_coefficient = 0.0470526;
		constexpr double angular_speed = 22.601;
		constexpr double torque = 30.8089;
		constexpr double expected = 700.74;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 123") {
		constexpr double hysteresis_loss = 4.29219;
		constexpr double eddy_current_loss_coefficient = 0.0460539;
		constexpr double angular_speed = 25.2632;
		constexpr double torque = 30.0582;
		constexpr double expected = 764.823;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 124") {
		constexpr double hysteresis_loss = 4.38538;
		constexpr double eddy_current_loss_coefficient = 0.028539;
		constexpr double angular_speed = -5.45093;
		constexpr double torque = 48.4766;
		constexpr double expected = -260.013;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 125") {
		constexpr double hysteresis_loss = 4.13836;
		constexpr double eddy_current_loss_coefficient = 0.019554;
		constexpr double angular_speed = 0.434717;
		constexpr double torque = 27.5708;
		constexpr double expected = 16.1324;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 126") {
		constexpr double hysteresis_loss = 2.97674;
		constexpr double eddy_current_loss_coefficient = 0.00601032;
		constexpr double angular_speed = -6.73824;
		constexpr double torque = 33.0537;
		constexpr double expected = -219.788;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 127") {
		constexpr double hysteresis_loss = 2.22652;
		constexpr double eddy_current_loss_coefficient = 0.0233127;
		constexpr double angular_speed = 4.32823;
		constexpr double torque = 58.1897;
		constexpr double expected = 254.186;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 128") {
		constexpr double hysteresis_loss = 4.87779;
		constexpr double eddy_current_loss_coefficient = 0.0264396;
		constexpr double angular_speed = -10.604;
		constexpr double torque = 12.5701;
		constexpr double expected = -128.696;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 129") {
		constexpr double hysteresis_loss = 1.92508;
		constexpr double eddy_current_loss_coefficient = 0.0413555;
		constexpr double angular_speed = 10.5432;
		constexpr double torque = 16.1968;
		constexpr double expected = 173.127;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 130") {
		constexpr double hysteresis_loss = 3.52189;
		constexpr double eddy_current_loss_coefficient = 0.0179551;
		constexpr double angular_speed = -1.78533;
		constexpr double torque = 24.7821;
		constexpr double expected = -40.7544;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 131") {
		constexpr double hysteresis_loss = 3.16717;
		constexpr double eddy_current_loss_coefficient = 0.0467693;
		constexpr double angular_speed = -18.3873;
		constexpr double torque = 49.022;
		constexpr double expected = -899.077;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 132") {
		constexpr double hysteresis_loss = 1.43992;
		constexpr double eddy_current_loss_coefficient = 0.0208164;
		constexpr double angular_speed = -2.18935;
		constexpr double torque = 31.2433;
		constexpr double expected = -67.0081;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 133") {
		constexpr double hysteresis_loss = 4.92565;
		constexpr double eddy_current_loss_coefficient = 0.0465755;
		constexpr double angular_speed = -3.54113;
		constexpr double torque = 16.6058;
		constexpr double expected = -54.0427;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 134") {
		constexpr double hysteresis_loss = 4.5677;
		constexpr double eddy_current_loss_coefficient = 0.0266534;
		constexpr double angular_speed = 1.71098;
		constexpr double torque = 84.2414;
		constexpr double expected = 148.748;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 135") {
		constexpr double hysteresis_loss = 4.29713;
		constexpr double eddy_current_loss_coefficient = 0.0119138;
		constexpr double angular_speed = -17.1449;
		constexpr double torque = 114.317;
		constexpr double expected = -1955.86;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 136") {
		constexpr double hysteresis_loss = 1.95515;
		constexpr double eddy_current_loss_coefficient = 0.0210825;
		constexpr double angular_speed = 7.52366;
		constexpr double torque = 106.692;
		constexpr double expected = 804.827;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 137") {
		constexpr double hysteresis_loss = 2.91836;
		constexpr double eddy_current_loss_coefficient = 0.0278383;
		constexpr double angular_speed = -6.43748;
		constexpr double torque = 43.8019;
		constexpr double expected = -279.235;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 138") {
		constexpr double hysteresis_loss = 2.11497;
		constexpr double eddy_current_loss_coefficient = 0.0103559;
		constexpr double angular_speed = -3.73318;
		constexpr double torque = 109.201;
		constexpr double expected = -405.593;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 139") {
		constexpr double hysteresis_loss = 2.88346;
		constexpr double eddy_current_loss_coefficient = 0.010788;
		constexpr double angular_speed = -8.60369;
		constexpr double torque = 67.1977;
		constexpr double expected = -575.358;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 140") {
		constexpr double hysteresis_loss = 3.41942;
		constexpr double eddy_current_loss_coefficient = 0.0494862;
		constexpr double angular_speed = 21.1975;
		constexpr double torque = 63.6635;
		constexpr double expected = 1353.98;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 141") {
		constexpr double hysteresis_loss = 4.92202;
		constexpr double eddy_current_loss_coefficient = 0.0177861;
		constexpr double angular_speed = -1.24796;
		constexpr double torque = 23.1111;
		constexpr double expected = -23.9418;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 142") {
		constexpr double hysteresis_loss = 3.1872;
		constexpr double eddy_current_loss_coefficient = 0.0212409;
		constexpr double angular_speed = -14.1238;
		constexpr double torque = 60.3887;
		constexpr double expected = -850.032;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 143") {
		constexpr double hysteresis_loss = 3.52302;
		constexpr double eddy_current_loss_coefficient = 0.0251106;
		constexpr double angular_speed = -25.2047;
		constexpr double torque = 62.4507;
		constexpr double expected = -1571.16;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 144") {
		constexpr double hysteresis_loss = 2.95681;
		constexpr double eddy_current_loss_coefficient = 0.0444095;
		constexpr double angular_speed = -23.5735;
		constexpr double torque = 107.302;
		constexpr double expected = -2527.58;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 145") {
		constexpr double hysteresis_loss = 3.00594;
		constexpr double eddy_current_loss_coefficient = 0.0366033;
		constexpr double angular_speed = 26.8458;
		constexpr double torque = 111.92;
		constexpr double expected = 3008.58;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 146") {
		constexpr double hysteresis_loss = 2.92516;
		constexpr double eddy_current_loss_coefficient = 0.0326739;
		constexpr double angular_speed = -8.85647;
		constexpr double torque = 22.3201;
		constexpr double expected = -195.041;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 147") {
		constexpr double hysteresis_loss = 3.98362;
		constexpr double eddy_current_loss_coefficient = 0.0149248;
		constexpr double angular_speed = 15.1377;
		constexpr double torque = 15.9589;
		constexpr double expected = 245.791;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 148") {
		constexpr double hysteresis_loss = 2.04436;
		constexpr double eddy_current_loss_coefficient = 0.0190736;
		constexpr double angular_speed = 1.06607;
		constexpr double torque = 92.6658;
		constexpr double expected = 100.853;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 149") {
		constexpr double hysteresis_loss = 4.7436;
		constexpr double eddy_current_loss_coefficient = 0.00639442;
		constexpr double angular_speed = 8.91352;
		constexpr double torque = 65.1493;
		constexpr double expected = 585.51;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 150") {
		constexpr double hysteresis_loss = 4.01827;
		constexpr double eddy_current_loss_coefficient = 0.0318713;
		constexpr double angular_speed = -28.0501;
		constexpr double torque = 120.596;
		constexpr double expected = -3379.6;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 151") {
		constexpr double hysteresis_loss = 2.05038;
		constexpr double eddy_current_loss_coefficient = 0.0220814;
		constexpr double angular_speed = 29.3432;
		constexpr double torque = 99.9621;
		constexpr double expected = 2935.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 152") {
		constexpr double hysteresis_loss = 4.93749;
		constexpr double eddy_current_loss_coefficient = 0.0212603;
		constexpr double angular_speed = -29.7332;
		constexpr double torque = 12.2253;
		constexpr double expected = -359.19;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 153") {
		constexpr double hysteresis_loss = 1.3724;
		constexpr double eddy_current_loss_coefficient = 0.0120683;
		constexpr double angular_speed = -10.9688;
		constexpr double torque = 24.1799;
		constexpr double expected = -263.985;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 154") {
		constexpr double hysteresis_loss = 3.73216;
		constexpr double eddy_current_loss_coefficient = 0.025122;
		constexpr double angular_speed = -9.85728;
		constexpr double torque = 21.6126;
		constexpr double expected = -209.557;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 155") {
		constexpr double hysteresis_loss = 2.46605;
		constexpr double eddy_current_loss_coefficient = 0.0469839;
		constexpr double angular_speed = 28.3976;
		constexpr double torque = 68.0735;
		constexpr double expected = 1936.93;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 156") {
		constexpr double hysteresis_loss = 4.40666;
		constexpr double eddy_current_loss_coefficient = 0.011796;
		constexpr double angular_speed = -5.04907;
		constexpr double torque = 78.0292;
		constexpr double expected = -389.628;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 157") {
		constexpr double hysteresis_loss = 1.00754;
		constexpr double eddy_current_loss_coefficient = 0.0202723;
		constexpr double angular_speed = 29.8102;
		constexpr double torque = 116.59;
		constexpr double expected = 3477.18;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 158") {
		constexpr double hysteresis_loss = 1.53575;
		constexpr double eddy_current_loss_coefficient = 0.0139364;
		constexpr double angular_speed = -19.2917;
		constexpr double torque = 36.8084;
		constexpr double expected = -708.83;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 159") {
		constexpr double hysteresis_loss = 4.83424;
		constexpr double eddy_current_loss_coefficient = 0.0233189;
		constexpr double angular_speed = -8.94317;
		constexpr double torque = 4.93599;
		constexpr double expected = -39.5177;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 160") {
		constexpr double hysteresis_loss = 1.27239;
		constexpr double eddy_current_loss_coefficient = 0.0402366;
		constexpr double angular_speed = 18.7962;
		constexpr double torque = 22.8882;
		constexpr double expected = 432.241;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 161") {
		constexpr double hysteresis_loss = 1.94439;
		constexpr double eddy_current_loss_coefficient = 0.0096284;
		constexpr double angular_speed = 14.3081;
		constexpr double torque = 74.8943;
		constexpr double expected = 1073.68;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 162") {
		constexpr double hysteresis_loss = 3.56307;
		constexpr double eddy_current_loss_coefficient = 0.0387699;
		constexpr double angular_speed = -2.67755;
		constexpr double torque = 9.4505;
		constexpr double expected = -21.8449;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 163") {
		constexpr double hysteresis_loss = 4.90887;
		constexpr double eddy_current_loss_coefficient = 0.0290194;
		constexpr double angular_speed = -19.3004;
		constexpr double torque = 103.402;
		constexpr double expected = -1991.35;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 164") {
		constexpr double hysteresis_loss = 2.71566;
		constexpr double eddy_current_loss_coefficient = 0.0275979;
		constexpr double angular_speed = -23.9447;
		constexpr double torque = 87.6269;
		constexpr double expected = -2096.15;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 165") {
		constexpr double hysteresis_loss = 1.23226;
		constexpr double eddy_current_loss_coefficient = 0.0186128;
		constexpr double angular_speed = 10.4143;
		constexpr double torque = 116.282;
		constexpr double expected = 1212.43;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 166") {
		constexpr double hysteresis_loss = 3.7775;
		constexpr double eddy_current_loss_coefficient = 0.0445045;
		constexpr double angular_speed = 0.199064;
		constexpr double torque = 77.1853;
		constexpr double expected = 19.1512;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 167") {
		constexpr double hysteresis_loss = 2.05147;
		constexpr double eddy_current_loss_coefficient = 0.0382294;
		constexpr double angular_speed = 11.7069;
		constexpr double torque = 111.781;
		constexpr double expected = 1311.1;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 168") {
		constexpr double hysteresis_loss = 3.92792;
		constexpr double eddy_current_loss_coefficient = 0.00139668;
		constexpr double angular_speed = 17.868;
		constexpr double torque = 87.1697;
		constexpr double expected = 1561.5;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 169") {
		constexpr double hysteresis_loss = 2.33105;
		constexpr double eddy_current_loss_coefficient = 0.0406567;
		constexpr double angular_speed = 27.2745;
		constexpr double torque = 31.3093;
		constexpr double expected = 857.385;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 170") {
		constexpr double hysteresis_loss = 2.78657;
		constexpr double eddy_current_loss_coefficient = 0.0233469;
		constexpr double angular_speed = 5.39107;
		constexpr double torque = 44.5554;
		constexpr double expected = 243.114;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 171") {
		constexpr double hysteresis_loss = 2.61593;
		constexpr double eddy_current_loss_coefficient = 0.0499273;
		constexpr double angular_speed = 3.85771;
		constexpr double torque = 117.146;
		constexpr double expected = 454.724;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 172") {
		constexpr double hysteresis_loss = 2.575;
		constexpr double eddy_current_loss_coefficient = 0.00100528;
		constexpr double angular_speed = -0.204931;
		constexpr double torque = 101.152;
		constexpr double expected = -18.1543;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 173") {
		constexpr double hysteresis_loss = 3.36948;
		constexpr double eddy_current_loss_coefficient = 0.0268906;
		constexpr double angular_speed = -16.491;
		constexpr double torque = 25.7234;
		constexpr double expected = -421.279;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 174") {
		constexpr double hysteresis_loss = 2.71603;
		constexpr double eddy_current_loss_coefficient = 0.0471749;
		constexpr double angular_speed = -18.551;
		constexpr double torque = 92.9658;
		constexpr double expected = -1722.76;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 175") {
		constexpr double hysteresis_loss = 1.17285;
		constexpr double eddy_current_loss_coefficient = 0.0224991;
		constexpr double angular_speed = -10.2825;
		constexpr double torque = 43.0527;
		constexpr double expected = -441.746;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 176") {
		constexpr double hysteresis_loss = 4.42851;
		constexpr double eddy_current_loss_coefficient = 0.0054828;
		constexpr double angular_speed = 28.2574;
		constexpr double torque = 124.978;
		constexpr double expected = 3536.14;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 177") {
		constexpr double hysteresis_loss = 3.6994;
		constexpr double eddy_current_loss_coefficient = 0.0239118;
		constexpr double angular_speed = 11.2252;
		constexpr double torque = 90.8138;
		constexpr double expected = 1023.37;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 178") {
		constexpr double hysteresis_loss = 2.71119;
		constexpr double eddy_current_loss_coefficient = 0.0144268;
		constexpr double angular_speed = -15.0231;
		constexpr double torque = 19.2739;
		constexpr double expected = -287.06;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 179") {
		constexpr double hysteresis_loss = 4.35727;
		constexpr double eddy_current_loss_coefficient = 0.0222141;
		constexpr double angular_speed = 16.7488;
		constexpr double torque = 31.627;
		constexpr double expected = 534.444;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 180") {
		constexpr double hysteresis_loss = 4.74427;
		constexpr double eddy_current_loss_coefficient = 0.0271045;
		constexpr double angular_speed = 15.9756;
		constexpr double torque = 107.31;
		constexpr double expected = 1719.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 181") {
		constexpr double hysteresis_loss = 4.06444;
		constexpr double eddy_current_loss_coefficient = 0.00909583;
		constexpr double angular_speed = -22.4833;
		constexpr double torque = 38.307;
		constexpr double expected = -857.407;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 182") {
		constexpr double hysteresis_loss = 3.19489;
		constexpr double eddy_current_loss_coefficient = 0.0484422;
		constexpr double angular_speed = -4.52884;
		constexpr double torque = 4.28033;
		constexpr double expected = -16.4094;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 183") {
		constexpr double hysteresis_loss = 2.90707;
		constexpr double eddy_current_loss_coefficient = 0.0088185;
		constexpr double angular_speed = -8.30382;
		constexpr double torque = 36.4002;
		constexpr double expected = -299.426;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 184") {
		constexpr double hysteresis_loss = 4.83497;
		constexpr double eddy_current_loss_coefficient = 0.0122299;
		constexpr double angular_speed = 3.86352;
		constexpr double torque = 0.139732;
		constexpr double expected = 5.42208;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 185") {
		constexpr double hysteresis_loss = 2.20027;
		constexpr double eddy_current_loss_coefficient = 0.0254622;
		constexpr double angular_speed = 25.0393;
		constexpr double torque = 113.041;
		constexpr double expected = 2833.31;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 186") {
		constexpr double hysteresis_loss = 1.04707;
		constexpr double eddy_current_loss_coefficient = 0.0321617;
		constexpr double angular_speed = -2.18553;
		constexpr double torque = 9.41351;
		constexpr double expected = -19.5967;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 187") {
		constexpr double hysteresis_loss = 2.31965;
		constexpr double eddy_current_loss_coefficient = 0.0236362;
		constexpr double angular_speed = -24.6358;
		constexpr double torque = 59.7337;
		constexpr double expected = -1469.85;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 188") {
		constexpr double hysteresis_loss = 4.71621;
		constexpr double eddy_current_loss_coefficient = 0.00762612;
		constexpr double angular_speed = -3.90604;
		constexpr double torque = 102.728;
		constexpr double expected = -396.573;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 189") {
		constexpr double hysteresis_loss = 2.31778;
		constexpr double eddy_current_loss_coefficient = 0.00755118;
		constexpr double angular_speed = 28.0578;
		constexpr double torque = 14.8431;
		constexpr double expected = 418.995;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 190") {
		constexpr double hysteresis_loss = 2.23565;
		constexpr double eddy_current_loss_coefficient = 0.0460189;
		constexpr double angular_speed = 8.22882;
		constexpr double torque = 123.297;
		constexpr double expected = 1017.21;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 191") {
		constexpr double hysteresis_loss = 2.8346;
		constexpr double eddy_current_loss_coefficient = 0.0292733;
		constexpr double angular_speed = -16.8322;
		constexpr double torque = 59.1445;
		constexpr double expected = -993.187;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 192") {
		constexpr double hysteresis_loss = 1.02196;
		constexpr double eddy_current_loss_coefficient = 0.00451986;
		constexpr double angular_speed = 22.254;
		constexpr double torque = 65.5159;
		constexpr double expected = 1459.11;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 193") {
		constexpr double hysteresis_loss = 4.41681;
		constexpr double eddy_current_loss_coefficient = 0.00722464;
		constexpr double angular_speed = 0.845712;
		constexpr double torque = 40.3562;
		constexpr double expected = 38.5527;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 194") {
		constexpr double hysteresis_loss = 3.26084;
		constexpr double eddy_current_loss_coefficient = 0.0113697;
		constexpr double angular_speed = 14.7811;
		constexpr double torque = 103.014;
		constexpr double expected = 1526.08;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 195") {
		constexpr double hysteresis_loss = 3.45358;
		constexpr double eddy_current_loss_coefficient = 0.0151891;
		constexpr double angular_speed = -14.9521;
		constexpr double torque = 34.4471;
		constexpr double expected = -511.832;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 196") {
		constexpr double hysteresis_loss = 3.37389;
		constexpr double eddy_current_loss_coefficient = 0.0423554;
		constexpr double angular_speed = 10.7461;
		constexpr double torque = 33.8906;
		constexpr double expected = 368.02;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 197") {
		constexpr double hysteresis_loss = 2.21777;
		constexpr double eddy_current_loss_coefficient = 0.0451817;
		constexpr double angular_speed = 12.652;
		constexpr double torque = 3.1063;
		constexpr double expected = 42.0905;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 198") {
		constexpr double hysteresis_loss = 4.94968;
		constexpr double eddy_current_loss_coefficient = 0.0288109;
		constexpr double angular_speed = 9.7377;
		constexpr double torque = 95.2318;
		constexpr double expected = 932.569;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 199") {
		constexpr double hysteresis_loss = 3.40342;
		constexpr double eddy_current_loss_coefficient = 0.00482061;
		constexpr double angular_speed = -16.6155;
		constexpr double torque = 92.3477;
		constexpr double expected = -1531.08;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 200") {
		constexpr double hysteresis_loss = 2.97421;
		constexpr double eddy_current_loss_coefficient = 0.0299024;
		constexpr double angular_speed = 0.516235;
		constexpr double torque = 120.165;
		constexpr double expected = 65.0231;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 201") {
		constexpr double hysteresis_loss = 1.12803;
		constexpr double eddy_current_loss_coefficient = 0.00319434;
		constexpr double angular_speed = 9.78415;
		constexpr double torque = 26.5413;
		constexpr double expected = 260.843;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 202") {
		constexpr double hysteresis_loss = 1.79276;
		constexpr double eddy_current_loss_coefficient = 0.0317538;
		constexpr double angular_speed = -24.6352;
		constexpr double torque = 72.0769;
		constexpr double expected = -1774.62;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 203") {
		constexpr double hysteresis_loss = 1.68825;
		constexpr double eddy_current_loss_coefficient = 0.0357148;
		constexpr double angular_speed = 0.416644;
		constexpr double torque = 89.4988;
		constexpr double expected = 38.9922;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 204") {
		constexpr double hysteresis_loss = 2.64553;
		constexpr double eddy_current_loss_coefficient = 0.031604;
		constexpr double angular_speed = 25.6245;
		constexpr double torque = 20.5708;
		constexpr double expected = 530.572;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 205") {
		constexpr double hysteresis_loss = 4.2719;
		constexpr double eddy_current_loss_coefficient = 0.0410866;
		constexpr double angular_speed = -29.4912;
		constexpr double torque = 10.3587;
		constexpr double expected = -302.432;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 206") {
		constexpr double hysteresis_loss = 4.86912;
		constexpr double eddy_current_loss_coefficient = 0.041051;
		constexpr double angular_speed = -2.88495;
		constexpr double torque = 18.9055;
		constexpr double expected = -49.7908;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 207") {
		constexpr double hysteresis_loss = 3.28674;
		constexpr double eddy_current_loss_coefficient = 0.0151136;
		constexpr double angular_speed = 3.78205;
		constexpr double torque = 55.9239;
		constexpr double expected = 214.851;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 208") {
		constexpr double hysteresis_loss = 4.16592;
		constexpr double eddy_current_loss_coefficient = 0.0317111;
		constexpr double angular_speed = -23.6385;
		constexpr double torque = 50.9665;
		constexpr double expected = -1201.36;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 209") {
		constexpr double hysteresis_loss = 2.82555;
		constexpr double eddy_current_loss_coefficient = 0.0276528;
		constexpr double angular_speed = 21.7071;
		constexpr double torque = 45.9345;
		constexpr double expected = 1000.53;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 210") {
		constexpr double hysteresis_loss = 1.27372;
		constexpr double eddy_current_loss_coefficient = 0.0457365;
		constexpr double angular_speed = 1.25427;
		constexpr double torque = 57.5573;
		constexpr double expected = 73.5236;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 211") {
		constexpr double hysteresis_loss = 4.07033;
		constexpr double eddy_current_loss_coefficient = 0.0287754;
		constexpr double angular_speed = -10.697;
		constexpr double torque = 28.7365;
		constexpr double expected = -303.632;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 212") {
		constexpr double hysteresis_loss = 1.08386;
		constexpr double eddy_current_loss_coefficient = 0.00866632;
		constexpr double angular_speed = -17.9058;
		constexpr double torque = 78.2447;
		constexpr double expected = -1400.11;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 213") {
		constexpr double hysteresis_loss = 3.97462;
		constexpr double eddy_current_loss_coefficient = 0.0172546;
		constexpr double angular_speed = 24.7411;
		constexpr double torque = 61.3514;
		constexpr double expected = 1522.3;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 214") {
		constexpr double hysteresis_loss = 2.89929;
		constexpr double eddy_current_loss_coefficient = 0.0100117;
		constexpr double angular_speed = 17.4878;
		constexpr double torque = 115.373;
		constexpr double expected = 2020.69;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 215") {
		constexpr double hysteresis_loss = 2.06803;
		constexpr double eddy_current_loss_coefficient = 0.0185736;
		constexpr double angular_speed = -11.5237;
		constexpr double torque = 15.1823;
		constexpr double expected = -173.102;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 216") {
		constexpr double hysteresis_loss = 2.98656;
		constexpr double eddy_current_loss_coefficient = 0.0100561;
		constexpr double angular_speed = 5.03779;
		constexpr double torque = 24.6362;
		constexpr double expected = 127.15;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 217") {
		constexpr double hysteresis_loss = 4.91429;
		constexpr double eddy_current_loss_coefficient = 0.0257312;
		constexpr double angular_speed = -16.1164;
		constexpr double torque = 122.042;
		constexpr double expected = -1962.38;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 218") {
		constexpr double hysteresis_loss = 4.39217;
		constexpr double eddy_current_loss_coefficient = 0.0249585;
		constexpr double angular_speed = 11.9935;
		constexpr double torque = 51.0546;
		constexpr double expected = 617.015;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 219") {
		constexpr double hysteresis_loss = 3.117;
		constexpr double eddy_current_loss_coefficient = 0.00935427;
		constexpr double angular_speed = 24.3131;
		constexpr double torque = 103.742;
		constexpr double expected = 2525.62;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 220") {
		constexpr double hysteresis_loss = 1.63866;
		constexpr double eddy_current_loss_coefficient = 0.0165708;
		constexpr double angular_speed = 11.0308;
		constexpr double torque = 60.1876;
		constexpr double expected = 665.739;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 221") {
		constexpr double hysteresis_loss = 1.70104;
		constexpr double eddy_current_loss_coefficient = 0.016951;
		constexpr double angular_speed = 19.9658;
		constexpr double torque = 24.7238;
		constexpr double expected = 495.669;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 222") {
		constexpr double hysteresis_loss = 3.44919;
		constexpr double eddy_current_loss_coefficient = 0.0189402;
		constexpr double angular_speed = 15.7628;
		constexpr double torque = 44.5569;
		constexpr double expected = 706.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 223") {
		constexpr double hysteresis_loss = 4.76699;
		constexpr double eddy_current_loss_coefficient = 0.0257648;
		constexpr double angular_speed = -15.744;
		constexpr double torque = 120.333;
		constexpr double expected = -1890.17;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 224") {
		constexpr double hysteresis_loss = 1.04371;
		constexpr double eddy_current_loss_coefficient = 0.0127288;
		constexpr double angular_speed = -27.7006;
		constexpr double torque = 61.7034;
		constexpr double expected = -1708.53;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 225") {
		constexpr double hysteresis_loss = 1.39912;
		constexpr double eddy_current_loss_coefficient = 0.0410929;
		constexpr double angular_speed = -14.2046;
		constexpr double torque = 92.398;
		constexpr double expected = -1311.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 226") {
		constexpr double hysteresis_loss = 2.43889;
		constexpr double eddy_current_loss_coefficient = 0.0322493;
		constexpr double angular_speed = 24.6257;
		constexpr double torque = 54.2739;
		constexpr double expected = 1339.76;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 227") {
		constexpr double hysteresis_loss = 4.69236;
		constexpr double eddy_current_loss_coefficient = 0.0086307;
		constexpr double angular_speed = -17.269;
		constexpr double torque = 95.5237;
		constexpr double expected = -1645.06;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 228") {
		constexpr double hysteresis_loss = 4.07451;
		constexpr double eddy_current_loss_coefficient = 0.00393764;
		constexpr double angular_speed = -12.2212;
		constexpr double torque = 121.992;
		constexpr double expected = -1486.86;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 229") {
		constexpr double hysteresis_loss = 2.70865;
		constexpr double eddy_current_loss_coefficient = 0.0282044;
		constexpr double angular_speed = 0.700928;
		constexpr double torque = 40.3658;
		constexpr double expected = 31.022;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 230") {
		constexpr double hysteresis_loss = 2.84122;
		constexpr double eddy_current_loss_coefficient = 0.00667876;
		constexpr double angular_speed = 20.3807;
		constexpr double torque = 70.0383;
		constexpr double expected = 1430.4;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 231") {
		constexpr double hysteresis_loss = 2.63844;
		constexpr double eddy_current_loss_coefficient = 0.032475;
		constexpr double angular_speed = -13.243;
		constexpr double torque = 11.7789;
		constexpr double expected = -153.779;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 232") {
		constexpr double hysteresis_loss = 4.21864;
		constexpr double eddy_current_loss_coefficient = 0.0137226;
		constexpr double angular_speed = 10.6631;
		constexpr double torque = 68.1679;
		constexpr double expected = 731.248;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 233") {
		constexpr double hysteresis_loss = 1.46085;
		constexpr double eddy_current_loss_coefficient = 0.0320078;
		constexpr double angular_speed = 9.50679;
		constexpr double torque = 105.514;
		constexpr double expected = 1004.87;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 234") {
		constexpr double hysteresis_loss = 4.76436;
		constexpr double eddy_current_loss_coefficient = 0.00994535;
		constexpr double angular_speed = 3.10362;
		constexpr double torque = 110.058;
		constexpr double expected = 346.375;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 235") {
		constexpr double hysteresis_loss = 4.12057;
		constexpr double eddy_current_loss_coefficient = 0.0266147;
		constexpr double angular_speed = -13.6113;
		constexpr double torque = 119.63;
		constexpr double expected = -1624.57;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 236") {
		constexpr double hysteresis_loss = 4.70667;
		constexpr double eddy_current_loss_coefficient = 0.0279366;
		constexpr double angular_speed = 7.84101;
		constexpr double torque = 30.4484;
		constexpr double expected = 243.672;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 237") {
		constexpr double hysteresis_loss = 1.50807;
		constexpr double eddy_current_loss_coefficient = 0.0205302;
		constexpr double angular_speed = 7.47637;
		constexpr double torque = 16.3841;
		constexpr double expected = 124.155;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 238") {
		constexpr double hysteresis_loss = 4.34188;
		constexpr double eddy_current_loss_coefficient = 0.024387;
		constexpr double angular_speed = 8.23092;
		constexpr double torque = 112.378;
		constexpr double expected = 929.513;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 239") {
		constexpr double hysteresis_loss = 3.0711;
		constexpr double eddy_current_loss_coefficient = 0.0194109;
		constexpr double angular_speed = 23.2777;
		constexpr double torque = 117.605;
		constexpr double expected = 2741.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 240") {
		constexpr double hysteresis_loss = 3.65251;
		constexpr double eddy_current_loss_coefficient = 0.0261916;
		constexpr double angular_speed = 12.8017;
		constexpr double torque = 104.124;
		constexpr double expected = 1336.95;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 241") {
		constexpr double hysteresis_loss = 4.50338;
		constexpr double eddy_current_loss_coefficient = 0.0492255;
		constexpr double angular_speed = 9.56096;
		constexpr double torque = 48.0882;
		constexpr double expected = 464.743;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 242") {
		constexpr double hysteresis_loss = 2.16469;
		constexpr double eddy_current_loss_coefficient = 0.00452781;
		constexpr double angular_speed = -18.9707;
		constexpr double torque = 71.2048;
		constexpr double expected = -1348.73;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 243") {
		constexpr double hysteresis_loss = 3.86113;
		constexpr double eddy_current_loss_coefficient = 0.0255473;
		constexpr double angular_speed = 5.25551;
		constexpr double torque = 43.3208;
		constexpr double expected = 231.668;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 244") {
		constexpr double hysteresis_loss = 3.60153;
		constexpr double eddy_current_loss_coefficient = 0.0309205;
		constexpr double angular_speed = -5.45771;
		constexpr double torque = 79.1275;
		constexpr double expected = -428.422;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 245") {
		constexpr double hysteresis_loss = 4.96491;
		constexpr double eddy_current_loss_coefficient = 0.0438396;
		constexpr double angular_speed = -2.82012;
		constexpr double torque = 41.6168;
		constexpr double expected = -112.523;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 246") {
		constexpr double hysteresis_loss = 3.91344;
		constexpr double eddy_current_loss_coefficient = 0.0389771;
		constexpr double angular_speed = -1.03854;
		constexpr double torque = 80.7897;
		constexpr double expected = -80.0307;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 247") {
		constexpr double hysteresis_loss = 3.52958;
		constexpr double eddy_current_loss_coefficient = 0.0168362;
		constexpr double angular_speed = 8.20567;
		constexpr double torque = 29.3469;
		constexpr double expected = 244.479;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 248") {
		constexpr double hysteresis_loss = 2.01202;
		constexpr double eddy_current_loss_coefficient = 0.00546213;
		constexpr double angular_speed = 22.9479;
		constexpr double torque = 51.1874;
		constexpr double expected = 1176.78;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 249") {
		constexpr double hysteresis_loss = 1.91394;
		constexpr double eddy_current_loss_coefficient = 0.032019;
		constexpr double angular_speed = -18.5406;
		constexpr double torque = 3.96619;
		constexpr double expected = -72.2152;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 250") {
		constexpr double hysteresis_loss = 4.66063;
		constexpr double eddy_current_loss_coefficient = 0.0202097;
		constexpr double angular_speed = 17.661;
		constexpr double torque = 91.5702;
		constexpr double expected = 1622.24;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 251") {
		constexpr double hysteresis_loss = 4.45155;
		constexpr double eddy_current_loss_coefficient = 0.0374041;
		constexpr double angular_speed = -7.76095;
		constexpr double torque = 66.0418;
		constexpr double expected = -508.386;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 252") {
		constexpr double hysteresis_loss = 1.36954;
		constexpr double eddy_current_loss_coefficient = 0.0480244;
		constexpr double angular_speed = 25.985;
		constexpr double torque = 33.8776;
		constexpr double expected = 882.928;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 253") {
		constexpr double hysteresis_loss = 1.71548;
		constexpr double eddy_current_loss_coefficient = 0.0107086;
		constexpr double angular_speed = -28.2063;
		constexpr double torque = 102.178;
		constexpr double expected = -2880.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 254") {
		constexpr double hysteresis_loss = 3.44645;
		constexpr double eddy_current_loss_coefficient = 0.047077;
		constexpr double angular_speed = 5.13406;
		constexpr double torque = 120.437;
		constexpr double expected = 622.018;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 255") {
		constexpr double hysteresis_loss = 2.33101;
		constexpr double eddy_current_loss_coefficient = 0.0422278;
		constexpr double angular_speed = -25.0424;
		constexpr double torque = 115.614;
		constexpr double expected = -2893.98;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 256") {
		constexpr double hysteresis_loss = 4.205;
		constexpr double eddy_current_loss_coefficient = 0.0262073;
		constexpr double angular_speed = 1.5721;
		constexpr double torque = 42.6388;
		constexpr double expected = 71.2787;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 257") {
		constexpr double hysteresis_loss = 1.00812;
		constexpr double eddy_current_loss_coefficient = 0.0457469;
		constexpr double angular_speed = -22.5397;
		constexpr double torque = 113.342;
		constexpr double expected = -2554.72;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 258") {
		constexpr double hysteresis_loss = 3.1707;
		constexpr double eddy_current_loss_coefficient = 0.0224311;
		constexpr double angular_speed = 8.84417;
		constexpr double torque = 41.2031;
		constexpr double expected = 367.777;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 259") {
		constexpr double hysteresis_loss = 3.30662;
		constexpr double eddy_current_loss_coefficient = 0.0216237;
		constexpr double angular_speed = -26.2187;
		constexpr double torque = 51.0423;
		constexpr double expected = -1335.53;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 260") {
		constexpr double hysteresis_loss = 3.37908;
		constexpr double eddy_current_loss_coefficient = 0.00413412;
		constexpr double angular_speed = 4.39567;
		constexpr double torque = 70.0764;
		constexpr double expected = 311.431;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 261") {
		constexpr double hysteresis_loss = 4.95827;
		constexpr double eddy_current_loss_coefficient = 0.00628426;
		constexpr double angular_speed = 16.1839;
		constexpr double torque = 93.8765;
		constexpr double expected = 1524.35;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 262") {
		constexpr double hysteresis_loss = 4.53243;
		constexpr double eddy_current_loss_coefficient = 0.0360273;
		constexpr double angular_speed = 26.0561;
		constexpr double torque = 79.391;
		constexpr double expected = 2074.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 263") {
		constexpr double hysteresis_loss = 3.00403;
		constexpr double eddy_current_loss_coefficient = 0.0318385;
		constexpr double angular_speed = -4.13422;
		constexpr double torque = 95.5462;
		constexpr double expected = -392.137;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 264") {
		constexpr double hysteresis_loss = 1.50296;
		constexpr double eddy_current_loss_coefficient = 0.0372385;
		constexpr double angular_speed = -4.0395;
		constexpr double torque = 37.4381;
		constexpr double expected = -149.879;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 265") {
		constexpr double hysteresis_loss = 2.48573;
		constexpr double eddy_current_loss_coefficient = 0.01241;
		constexpr double angular_speed = 14.3436;
		constexpr double torque = 8.89068;
		constexpr double expected = 130.188;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 266") {
		constexpr double hysteresis_loss = 4.93133;
		constexpr double eddy_current_loss_coefficient = 0.018323;
		constexpr double angular_speed = 4.15967;
		constexpr double torque = 56.7074;
		constexpr double expected = 240.892;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 267") {
		constexpr double hysteresis_loss = 3.5178;
		constexpr double eddy_current_loss_coefficient = 0.00205743;
		constexpr double angular_speed = -12.8307;
		constexpr double torque = 110.308;
		constexpr double expected = -1411.84;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 268") {
		constexpr double hysteresis_loss = 2.58532;
		constexpr double eddy_current_loss_coefficient = 0.0168267;
		constexpr double angular_speed = -29.0503;
		constexpr double torque = 74.4713;
		constexpr double expected = -2161.32;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 269") {
		constexpr double hysteresis_loss = 3.37925;
		constexpr double eddy_current_loss_coefficient = 0.0214406;
		constexpr double angular_speed = -13.9563;
		constexpr double torque = 115.252;
		constexpr double expected = -1605.41;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 270") {
		constexpr double hysteresis_loss = 2.59329;
		constexpr double eddy_current_loss_coefficient = 0.0152787;
		constexpr double angular_speed = -8.75782;
		constexpr double torque = 2.4106;
		constexpr double expected = -18.6521;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 271") {
		constexpr double hysteresis_loss = 4.57327;
		constexpr double eddy_current_loss_coefficient = 0.0292599;
		constexpr double angular_speed = 7.20415;
		constexpr double torque = 81.5162;
		constexpr double expected = 592.039;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 272") {
		constexpr double hysteresis_loss = 4.81144;
		constexpr double eddy_current_loss_coefficient = 0.0248566;
		constexpr double angular_speed = -1.04574;
		constexpr double torque = 88.2795;
		constexpr double expected = -87.5322;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 273") {
		constexpr double hysteresis_loss = 2.34424;
		constexpr double eddy_current_loss_coefficient = 0.0294552;
		constexpr double angular_speed = 10.5237;
		constexpr double torque = 61.9456;
		constexpr double expected = 654.553;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 274") {
		constexpr double hysteresis_loss = 4.701;
		constexpr double eddy_current_loss_coefficient = 0.0196911;
		constexpr double angular_speed = 25.3102;
		constexpr double torque = 44.644;
		constexpr double expected = 1135.15;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 275") {
		constexpr double hysteresis_loss = 3.01003;
		constexpr double eddy_current_loss_coefficient = 0.0356431;
		constexpr double angular_speed = -12.9489;
		constexpr double torque = 0.564508;
		constexpr double expected = -4.76125;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 276") {
		constexpr double hysteresis_loss = 3.48435;
		constexpr double eddy_current_loss_coefficient = 0.00851379;
		constexpr double angular_speed = 24.5832;
		constexpr double torque = 11.2206;
		constexpr double expected = 279.533;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 277") {
		constexpr double hysteresis_loss = 1.06472;
		constexpr double eddy_current_loss_coefficient = 0.0076587;
		constexpr double angular_speed = 10.0102;
		constexpr double torque = 51.5012;
		constexpr double expected = 516.681;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 278") {
		constexpr double hysteresis_loss = 4.34021;
		constexpr double eddy_current_loss_coefficient = 0.00551177;
		constexpr double angular_speed = -16.1337;
		constexpr double torque = 51.6093;
		constexpr double expected = -828.397;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 279") {
		constexpr double hysteresis_loss = 3.24891;
		constexpr double eddy_current_loss_coefficient = 0.0197771;
		constexpr double angular_speed = 20.5964;
		constexpr double torque = 113.588;
		constexpr double expected = 2343.17;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 280") {
		constexpr double hysteresis_loss = 4.67286;
		constexpr double eddy_current_loss_coefficient = 0.0468005;
		constexpr double angular_speed = 1.68039;
		constexpr double torque = 28.6056;
		constexpr double expected = 52.8201;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 281") {
		constexpr double hysteresis_loss = 1.1846;
		constexpr double eddy_current_loss_coefficient = 0.0430419;
		constexpr double angular_speed = -7.93138;
		constexpr double torque = 95.1186;
		constexpr double expected = -753.578;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 282") {
		constexpr double hysteresis_loss = 3.64882;
		constexpr double eddy_current_loss_coefficient = 0.0130944;
		constexpr double angular_speed = -16.6434;
		constexpr double torque = 81.9238;
		constexpr double expected = -1360.06;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 283") {
		constexpr double hysteresis_loss = 4.17;
		constexpr double eddy_current_loss_coefficient = 0.0365903;
		constexpr double angular_speed = 12.6698;
		constexpr double torque = 22.7381;
		constexpr double expected = 292.72;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 284") {
		constexpr double hysteresis_loss = 4.49257;
		constexpr double eddy_current_loss_coefficient = 0.0266936;
		constexpr double angular_speed = 9.72241;
		constexpr double torque = 105.801;
		constexpr double expected = 1033.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 285") {
		constexpr double hysteresis_loss = 3.21413;
		constexpr double eddy_current_loss_coefficient = 0.0401014;
		constexpr double angular_speed = -7.63419;
		constexpr double torque = 96.046;
		constexpr double expected = -730.325;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 286") {
		constexpr double hysteresis_loss = 3.9362;
		constexpr double eddy_current_loss_coefficient = 0.0236392;
		constexpr double angular_speed = 4.41981;
		constexpr double torque = 105.721;
		constexpr double expected = 471.308;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 287") {
		constexpr double hysteresis_loss = 4.97648;
		constexpr double eddy_current_loss_coefficient = 0.00905881;
		constexpr double angular_speed = 6.95741;
		constexpr double torque = 49.5664;
		constexpr double expected = 349.893;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 288") {
		constexpr double hysteresis_loss = 4.57291;
		constexpr double eddy_current_loss_coefficient = 0.034132;
		constexpr double angular_speed = -15.113;
		constexpr double torque = 4.52224;
		constexpr double expected = -64.2874;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 289") {
		constexpr double hysteresis_loss = 3.93886;
		constexpr double eddy_current_loss_coefficient = 0.00764055;
		constexpr double angular_speed = -1.74844;
		constexpr double torque = 71.8827;
		constexpr double expected = -121.757;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 290") {
		constexpr double hysteresis_loss = 4.05796;
		constexpr double eddy_current_loss_coefficient = 0.00375158;
		constexpr double angular_speed = 19.3296;
		constexpr double torque = 59.4919;
		constexpr double expected = 1154.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 291") {
		constexpr double hysteresis_loss = 1.78083;
		constexpr double eddy_current_loss_coefficient = 0.0147568;
		constexpr double angular_speed = -13.1564;
		constexpr double torque = 24.9712;
		constexpr double expected = -326.945;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 292") {
		constexpr double hysteresis_loss = 4.3555;
		constexpr double eddy_current_loss_coefficient = 0.0394542;
		constexpr double angular_speed = -13.9323;
		constexpr double torque = 119.979;
		constexpr double expected = -1667.79;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 293") {
		constexpr double hysteresis_loss = 2.44239;
		constexpr double eddy_current_loss_coefficient = 0.0123864;
		constexpr double angular_speed = -0.326077;
		constexpr double torque = 40.1784;
		constexpr double expected = -10.6629;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 294") {
		constexpr double hysteresis_loss = 4.62895;
		constexpr double eddy_current_loss_coefficient = 0.037198;
		constexpr double angular_speed = -5.99421;
		constexpr double torque = 27.3422;
		constexpr double expected = -159.489;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 295") {
		constexpr double hysteresis_loss = 1.71026;
		constexpr double eddy_current_loss_coefficient = 0.0188505;
		constexpr double angular_speed = -28.0207;
		constexpr double torque = 49.2823;
		constexpr double expected = -1379.74;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 296") {
		constexpr double hysteresis_loss = 1.87787;
		constexpr double eddy_current_loss_coefficient = 0.00643853;
		constexpr double angular_speed = 0.927694;
		constexpr double torque = 42.1205;
		constexpr double expected = 40.9587;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 297") {
		constexpr double hysteresis_loss = 2.33368;
		constexpr double eddy_current_loss_coefficient = 0.0129091;
		constexpr double angular_speed = -0.297202;
		constexpr double torque = 32.4515;
		constexpr double expected = -7.3148;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 298") {
		constexpr double hysteresis_loss = 4.91486;
		constexpr double eddy_current_loss_coefficient = 0.034647;
		constexpr double angular_speed = 10.3894;
		constexpr double torque = 94.672;
		constexpr double expected = 988.858;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 299") {
		constexpr double hysteresis_loss = 1.52845;
		constexpr double eddy_current_loss_coefficient = 0.0421893;
		constexpr double angular_speed = -5.04603;
		constexpr double torque = 13.1033;
		constexpr double expected = -64.8042;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 300") {
		constexpr double hysteresis_loss = 4.18855;
		constexpr double eddy_current_loss_coefficient = 0.0438454;
		constexpr double angular_speed = 27.3474;
		constexpr double torque = 115.921;
		constexpr double expected = 3175.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 301") {
		constexpr double hysteresis_loss = 2.18816;
		constexpr double eddy_current_loss_coefficient = 0.0438331;
		constexpr double angular_speed = 27.6512;
		constexpr double torque = 26.1904;
		constexpr double expected = 727.598;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 302") {
		constexpr double hysteresis_loss = 3.41997;
		constexpr double eddy_current_loss_coefficient = 0.0229311;
		constexpr double angular_speed = 23.3931;
		constexpr double torque = 18.3639;
		constexpr double expected = 433.546;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 303") {
		constexpr double hysteresis_loss = 1.3211;
		constexpr double eddy_current_loss_coefficient = 0.0448188;
		constexpr double angular_speed = 22.4295;
		constexpr double torque = 12.1427;
		constexpr double expected = 274.682;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 304") {
		constexpr double hysteresis_loss = 4.87416;
		constexpr double eddy_current_loss_coefficient = 0.00183958;
		constexpr double angular_speed = 22.4258;
		constexpr double torque = 57.3657;
		constexpr double expected = 1291.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 305") {
		constexpr double hysteresis_loss = 1.13995;
		constexpr double eddy_current_loss_coefficient = 0.0136044;
		constexpr double angular_speed = -23.8786;
		constexpr double torque = 79.7984;
		constexpr double expected = -1904.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 306") {
		constexpr double hysteresis_loss = 2.97472;
		constexpr double eddy_current_loss_coefficient = 0.0115541;
		constexpr double angular_speed = 6.39462;
		constexpr double torque = 34.4664;
		constexpr double expected = 223.448;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 307") {
		constexpr double hysteresis_loss = 2.29518;
		constexpr double eddy_current_loss_coefficient = 0.0476976;
		constexpr double angular_speed = 18.6405;
		constexpr double torque = 56.7216;
		constexpr double expected = 1060.5;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 308") {
		constexpr double hysteresis_loss = 4.32992;
		constexpr double eddy_current_loss_coefficient = 0.0423685;
		constexpr double angular_speed = -18.009;
		constexpr double torque = 84.1992;
		constexpr double expected = -1512.77;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 309") {
		constexpr double hysteresis_loss = 4.9948;
		constexpr double eddy_current_loss_coefficient = 0.0325598;
		constexpr double angular_speed = 18.4381;
		constexpr double torque = 68.2769;
		constexpr double expected = 1264.49;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 310") {
		constexpr double hysteresis_loss = 3.86605;
		constexpr double eddy_current_loss_coefficient = 0.0285527;
		constexpr double angular_speed = -22.8212;
		constexpr double torque = 112.433;
		constexpr double expected = -2562.64;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 311") {
		constexpr double hysteresis_loss = 1.21431;
		constexpr double eddy_current_loss_coefficient = 0.0486925;
		constexpr double angular_speed = -25.8097;
		constexpr double torque = 19.8912;
		constexpr double expected = -513.428;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 312") {
		constexpr double hysteresis_loss = 4.30398;
		constexpr double eddy_current_loss_coefficient = 0.0373916;
		constexpr double angular_speed = -24.3863;
		constexpr double torque = 122.738;
		constexpr double expected = -2989.74;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 313") {
		constexpr double hysteresis_loss = 1.37865;
		constexpr double eddy_current_loss_coefficient = 0.00343999;
		constexpr double angular_speed = -2.8942;
		constexpr double torque = 119.366;
		constexpr double expected = -344.101;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 314") {
		constexpr double hysteresis_loss = 2.89463;
		constexpr double eddy_current_loss_coefficient = 0.0208069;
		constexpr double angular_speed = -15.9669;
		constexpr double torque = 5.06428;
		constexpr double expected = -78.2986;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 315") {
		constexpr double hysteresis_loss = 1.05641;
		constexpr double eddy_current_loss_coefficient = 0.0380984;
		constexpr double angular_speed = -9.06058;
		constexpr double torque = 63.0201;
		constexpr double expected = -570.287;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 316") {
		constexpr double hysteresis_loss = 2.80479;
		constexpr double eddy_current_loss_coefficient = 0.0350013;
		constexpr double angular_speed = 12.0129;
		constexpr double torque = 30.1294;
		constexpr double expected = 365.167;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 317") {
		constexpr double hysteresis_loss = 1.98733;
		constexpr double eddy_current_loss_coefficient = 0.00827028;
		constexpr double angular_speed = 24.0186;
		constexpr double torque = 76.315;
		constexpr double expected = 1835.17;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 318") {
		constexpr double hysteresis_loss = 4.67158;
		constexpr double eddy_current_loss_coefficient = 0.018278;
		constexpr double angular_speed = 23.7092;
		constexpr double torque = 82.371;
		constexpr double expected = 1958.05;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 319") {
		constexpr double hysteresis_loss = 2.78692;
		constexpr double eddy_current_loss_coefficient = 0.0222183;
		constexpr double angular_speed = -2.06451;
		constexpr double torque = 84.8406;
		constexpr double expected = -172.413;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 320") {
		constexpr double hysteresis_loss = 4.59298;
		constexpr double eddy_current_loss_coefficient = 0.0369008;
		constexpr double angular_speed = -9.72919;
		constexpr double torque = 83.516;
		constexpr double expected = -808.309;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 321") {
		constexpr double hysteresis_loss = 4.6946;
		constexpr double eddy_current_loss_coefficient = 0.031098;
		constexpr double angular_speed = -12.165;
		constexpr double torque = 97.9065;
		constexpr double expected = -1186.71;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 322") {
		constexpr double hysteresis_loss = 3.83788;
		constexpr double eddy_current_loss_coefficient = 0.0206953;
		constexpr double angular_speed = -22.4728;
		constexpr double torque = 13.8494;
		constexpr double expected = -307.861;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 323") {
		constexpr double hysteresis_loss = 3.57362;
		constexpr double eddy_current_loss_coefficient = 0.00850041;
		constexpr double angular_speed = -23.6629;
		constexpr double torque = 102.284;
		constexpr double expected = -2416.97;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 324") {
		constexpr double hysteresis_loss = 2.74145;
		constexpr double eddy_current_loss_coefficient = 0.0119866;
		constexpr double angular_speed = -29.1308;
		constexpr double torque = 34.2836;
		constexpr double expected = -996.318;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 325") {
		constexpr double hysteresis_loss = 4.28592;
		constexpr double eddy_current_loss_coefficient = 0.0424459;
		constexpr double angular_speed = 11.5402;
		constexpr double torque = 108.816;
		constexpr double expected = 1260.53;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 326") {
		constexpr double hysteresis_loss = 4.27427;
		constexpr double eddy_current_loss_coefficient = 0.032078;
		constexpr double angular_speed = 23.6962;
		constexpr double torque = 90.3081;
		constexpr double expected = 2144.99;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 327") {
		constexpr double hysteresis_loss = 1.97935;
		constexpr double eddy_current_loss_coefficient = 0.0462788;
		constexpr double angular_speed = 15.1159;
		constexpr double torque = 94.5531;
		constexpr double expected = 1431.94;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 328") {
		constexpr double hysteresis_loss = 2.9804;
		constexpr double eddy_current_loss_coefficient = 0.0149353;
		constexpr double angular_speed = 6.88754;
		constexpr double torque = 63.1309;
		constexpr double expected = 437.9;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 329") {
		constexpr double hysteresis_loss = 1.41043;
		constexpr double eddy_current_loss_coefficient = 0.00970492;
		constexpr double angular_speed = 15.5724;
		constexpr double torque = 120.717;
		constexpr double expected = 1881.4;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 330") {
		constexpr double hysteresis_loss = 2.6013;
		constexpr double eddy_current_loss_coefficient = 0.00305247;
		constexpr double angular_speed = 14.6558;
		constexpr double torque = 22.4998;
		constexpr double expected = 332.398;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 331") {
		constexpr double hysteresis_loss = 3.61124;
		constexpr double eddy_current_loss_coefficient = 0.0448175;
		constexpr double angular_speed = 20.9228;
		constexpr double torque = 45.4426;
		constexpr double expected = 955.337;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 332") {
		constexpr double hysteresis_loss = 1.23073;
		constexpr double eddy_current_loss_coefficient = 0.0448814;
		constexpr double angular_speed = -7.47426;
		constexpr double torque = 86.8393;
		constexpr double expected = -648.165;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 333") {
		constexpr double hysteresis_loss = 3.8079;
		constexpr double eddy_current_loss_coefficient = 0.0183443;
		constexpr double angular_speed = -23.9167;
		constexpr double torque = 62.3226;
		constexpr double expected = -1487.18;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 334") {
		constexpr double hysteresis_loss = 2.11412;
		constexpr double eddy_current_loss_coefficient = 0.0271843;
		constexpr double angular_speed = 9.192;
		constexpr double torque = 27.8138;
		constexpr double expected = 258.028;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 335") {
		constexpr double hysteresis_loss = 3.11613;
		constexpr double eddy_current_loss_coefficient = 0.0402942;
		constexpr double angular_speed = -28.8832;
		constexpr double torque = 52.9536;
		constexpr double expected = -1527.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 336") {
		constexpr double hysteresis_loss = 4.03642;
		constexpr double eddy_current_loss_coefficient = 0.0206194;
		constexpr double angular_speed = -11.2328;
		constexpr double torque = 66.7633;
		constexpr double expected = -746.135;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 337") {
		constexpr double hysteresis_loss = 2.68654;
		constexpr double eddy_current_loss_coefficient = 0.0452038;
		constexpr double angular_speed = 9.92256;
		constexpr double torque = 113.7;
		constexpr double expected = 1131.33;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 338") {
		constexpr double hysteresis_loss = 1.3998;
		constexpr double eddy_current_loss_coefficient = 0.0390244;
		constexpr double angular_speed = 9.17524;
		constexpr double torque = 119.636;
		constexpr double expected = 1099.45;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 339") {
		constexpr double hysteresis_loss = 1.31263;
		constexpr double eddy_current_loss_coefficient = 0.0193159;
		constexpr double angular_speed = 18.7392;
		constexpr double torque = 112.806;
		constexpr double expected = 2115.57;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 340") {
		constexpr double hysteresis_loss = 2.95199;
		constexpr double eddy_current_loss_coefficient = 0.00560709;
		constexpr double angular_speed = 18.479;
		constexpr double torque = 12.6009;
		constexpr double expected = 235.908;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 341") {
		constexpr double hysteresis_loss = 2.30143;
		constexpr double eddy_current_loss_coefficient = 0.0261934;
		constexpr double angular_speed = 25.0676;
		constexpr double torque = 10.4416;
		constexpr double expected = 264.704;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 342") {
		constexpr double hysteresis_loss = 2.73827;
		constexpr double eddy_current_loss_coefficient = 0.0449303;
		constexpr double angular_speed = -22.7207;
		constexpr double torque = 44.5615;
		constexpr double expected = -1010.75;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 343") {
		constexpr double hysteresis_loss = 2.02922;
		constexpr double eddy_current_loss_coefficient = 0.011666;
		constexpr double angular_speed = 25.8549;
		constexpr double torque = 118.811;
		constexpr double expected = 3074.18;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 344") {
		constexpr double hysteresis_loss = 1.89482;
		constexpr double eddy_current_loss_coefficient = 0.0363516;
		constexpr double angular_speed = 6.93445;
		constexpr double torque = 14.5699;
		constexpr double expected = 103.181;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 345") {
		constexpr double hysteresis_loss = 1.77834;
		constexpr double eddy_current_loss_coefficient = 0.0164582;
		constexpr double angular_speed = 20.4127;
		constexpr double torque = 42.9207;
		constexpr double expected = 878.241;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 346") {
		constexpr double hysteresis_loss = 4.85581;
		constexpr double eddy_current_loss_coefficient = 0.0256761;
		constexpr double angular_speed = -14.8416;
		constexpr double torque = 8.261;
		constexpr double expected = -118.132;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 347") {
		constexpr double hysteresis_loss = 2.65311;
		constexpr double eddy_current_loss_coefficient = 0.0139493;
		constexpr double angular_speed = 1.6106;
		constexpr double torque = 78.0151;
		constexpr double expected = 128.327;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 348") {
		constexpr double hysteresis_loss = 1.27258;
		constexpr double eddy_current_loss_coefficient = 0.0168338;
		constexpr double angular_speed = 1.61473;
		constexpr double torque = 82.3296;
		constexpr double expected = 134.24;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 349") {
		constexpr double hysteresis_loss = 1.8249;
		constexpr double eddy_current_loss_coefficient = 0.0460886;
		constexpr double angular_speed = 1.80909;
		constexpr double torque = 46.622;
		constexpr double expected = 86.2514;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 350") {
		constexpr double hysteresis_loss = 2.54226;
		constexpr double eddy_current_loss_coefficient = 0.0084839;
		constexpr double angular_speed = 17.3649;
		constexpr double torque = 82.3403;
		constexpr double expected = 1432.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 351") {
		constexpr double hysteresis_loss = 4.4112;
		constexpr double eddy_current_loss_coefficient = 0.0222457;
		constexpr double angular_speed = 23.0217;
		constexpr double torque = 66.3736;
		constexpr double expected = 1532.96;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 352") {
		constexpr double hysteresis_loss = 1.9402;
		constexpr double eddy_current_loss_coefficient = 0.0458163;
		constexpr double angular_speed = -16.6519;
		constexpr double torque = 111.016;
		constexpr double expected = -1847.45;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 353") {
		constexpr double hysteresis_loss = 4.98011;
		constexpr double eddy_current_loss_coefficient = 0.0223075;
		constexpr double angular_speed = 23.0002;
		constexpr double torque = 82.6884;
		constexpr double expected = 1907.34;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 354") {
		constexpr double hysteresis_loss = 4.82694;
		constexpr double eddy_current_loss_coefficient = 0.0222004;
		constexpr double angular_speed = 29.6838;
		constexpr double torque = 110.156;
		constexpr double expected = 3275.32;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 355") {
		constexpr double hysteresis_loss = 4.04981;
		constexpr double eddy_current_loss_coefficient = 0.0409297;
		constexpr double angular_speed = 23.2544;
		constexpr double torque = 20.4798;
		constexpr double expected = 481.247;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 356") {
		constexpr double hysteresis_loss = 4.22464;
		constexpr double eddy_current_loss_coefficient = 0.0335341;
		constexpr double angular_speed = 16.6991;
		constexpr double torque = 60.4789;
		constexpr double expected = 1014.73;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 357") {
		constexpr double hysteresis_loss = 3.29969;
		constexpr double eddy_current_loss_coefficient = 0.0366502;
		constexpr double angular_speed = 22.7192;
		constexpr double torque = 99.4654;
		constexpr double expected = 2263.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 358") {
		constexpr double hysteresis_loss = 3.66918;
		constexpr double eddy_current_loss_coefficient = 0.0364128;
		constexpr double angular_speed = -6.84417;
		constexpr double torque = 44.7267;
		constexpr double expected = -302.698;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 359") {
		constexpr double hysteresis_loss = 1.38062;
		constexpr double eddy_current_loss_coefficient = 0.0315288;
		constexpr double angular_speed = -5.56968;
		constexpr double torque = 67.8123;
		constexpr double expected = -376.488;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 360") {
		constexpr double hysteresis_loss = 4.25256;
		constexpr double eddy_current_loss_coefficient = 0.0482055;
		constexpr double angular_speed = 10.0508;
		constexpr double torque = 5.18876;
		constexpr double expected = 56.8885;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 361") {
		constexpr double hysteresis_loss = 1.69097;
		constexpr double eddy_current_loss_coefficient = 0.0102807;
		constexpr double angular_speed = -5.92678;
		constexpr double torque = 83.4566;
		constexpr double expected = -492.999;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 362") {
		constexpr double hysteresis_loss = 2.80593;
		constexpr double eddy_current_loss_coefficient = 0.0232194;
		constexpr double angular_speed = 1.36525;
		constexpr double torque = 74.2265;
		constexpr double expected = 104.175;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 363") {
		constexpr double hysteresis_loss = 1.82632;
		constexpr double eddy_current_loss_coefficient = 0.0290103;
		constexpr double angular_speed = -4.81036;
		constexpr double torque = 12.5584;
		constexpr double expected = -58.7236;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 364") {
		constexpr double hysteresis_loss = 1.75239;
		constexpr double eddy_current_loss_coefficient = 0.020229;
		constexpr double angular_speed = -1.18838;
		constexpr double torque = 117.097;
		constexpr double expected = -137.428;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 365") {
		constexpr double hysteresis_loss = 3.4205;
		constexpr double eddy_current_loss_coefficient = 0.0118102;
		constexpr double angular_speed = 28.2091;
		constexpr double torque = 50.7732;
		constexpr double expected = 1436.02;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 366") {
		constexpr double hysteresis_loss = 1.12586;
		constexpr double eddy_current_loss_coefficient = 0.0215305;
		constexpr double angular_speed = 22.5279;
		constexpr double torque = 18.3985;
		constexpr double expected = 416.089;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 367") {
		constexpr double hysteresis_loss = 4.7935;
		constexpr double eddy_current_loss_coefficient = 0.0326738;
		constexpr double angular_speed = 22.2395;
		constexpr double torque = 24.5366;
		constexpr double expected = 551.202;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 368") {
		constexpr double hysteresis_loss = 2.2439;
		constexpr double eddy_current_loss_coefficient = 0.0134552;
		constexpr double angular_speed = -10.1954;
		constexpr double torque = 60.3583;
		constexpr double expected = -613.272;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 369") {
		constexpr double hysteresis_loss = 3.18557;
		constexpr double eddy_current_loss_coefficient = 0.0388379;
		constexpr double angular_speed = 12.6148;
		constexpr double torque = 80.6176;
		constexpr double expected = 1020.65;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 370") {
		constexpr double hysteresis_loss = 4.96876;
		constexpr double eddy_current_loss_coefficient = 0.0402596;
		constexpr double angular_speed = -4.48803;
		constexpr double torque = 57.4754;
		constexpr double expected = -253.163;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 371") {
		constexpr double hysteresis_loss = 4.77924;
		constexpr double eddy_current_loss_coefficient = 0.00533453;
		constexpr double angular_speed = 21.5915;
		constexpr double torque = 17.7674;
		constexpr double expected = 388.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 372") {
		constexpr double hysteresis_loss = 1.93272;
		constexpr double eddy_current_loss_coefficient = 0.00913441;
		constexpr double angular_speed = -18.6025;
		constexpr double torque = 5.817;
		constexpr double expected = -106.448;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 373") {
		constexpr double hysteresis_loss = 4.03176;
		constexpr double eddy_current_loss_coefficient = 0.0407736;
		constexpr double angular_speed = -19.8617;
		constexpr double torque = 27.2276;
		constexpr double expected = -537.565;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 374") {
		constexpr double hysteresis_loss = 1.38638;
		constexpr double eddy_current_loss_coefficient = 0.0339914;
		constexpr double angular_speed = -21.3841;
		constexpr double torque = 88.1564;
		constexpr double expected = -1884.48;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 375") {
		constexpr double hysteresis_loss = 1.7063;
		constexpr double eddy_current_loss_coefficient = 0.0467259;
		constexpr double angular_speed = 0.79524;
		constexpr double torque = 63.4351;
		constexpr double expected = 52.1896;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 376") {
		constexpr double hysteresis_loss = 4.10169;
		constexpr double eddy_current_loss_coefficient = 0.02166;
		constexpr double angular_speed = -13.2001;
		constexpr double torque = 55.9835;
		constexpr double expected = -735.172;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 377") {
		constexpr double hysteresis_loss = 2.54774;
		constexpr double eddy_current_loss_coefficient = 0.00325117;
		constexpr double angular_speed = 22.0278;
		constexpr double torque = 83.248;
		constexpr double expected = 1836.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 378") {
		constexpr double hysteresis_loss = 4.87009;
		constexpr double eddy_current_loss_coefficient = 0.0204842;
		constexpr double angular_speed = -3.66332;
		constexpr double torque = 120.557;
		constexpr double expected = -436.846;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 379") {
		constexpr double hysteresis_loss = 1.80688;
		constexpr double eddy_current_loss_coefficient = 0.0475472;
		constexpr double angular_speed = 28.5377;
		constexpr double torque = 15.5109;
		constexpr double expected = 445.808;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 380") {
		constexpr double hysteresis_loss = 2.63895;
		constexpr double eddy_current_loss_coefficient = 0.0414253;
		constexpr double angular_speed = -1.14802;
		constexpr double torque = 38.5469;
		constexpr double expected = -41.6614;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 381") {
		constexpr double hysteresis_loss = 4.93847;
		constexpr double eddy_current_loss_coefficient = 0.0390789;
		constexpr double angular_speed = -27.5153;
		constexpr double torque = 15.0144;
		constexpr double expected = -409.264;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 382") {
		constexpr double hysteresis_loss = 3.07443;
		constexpr double eddy_current_loss_coefficient = 0.00782755;
		constexpr double angular_speed = 25.3825;
		constexpr double torque = 60.2476;
		constexpr double expected = 1532.51;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 383") {
		constexpr double hysteresis_loss = 4.79333;
		constexpr double eddy_current_loss_coefficient = 0.01296;
		constexpr double angular_speed = 28.6682;
		constexpr double torque = 122.275;
		constexpr double expected = 3510.56;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 384") {
		constexpr double hysteresis_loss = 3.94929;
		constexpr double eddy_current_loss_coefficient = 0.0274486;
		constexpr double angular_speed = 16.753;
		constexpr double torque = 95.5037;
		constexpr double expected = 1604.38;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 385") {
		constexpr double hysteresis_loss = 2.29292;
		constexpr double eddy_current_loss_coefficient = 0.0125173;
		constexpr double angular_speed = 2.36374;
		constexpr double torque = 54.374;
		constexpr double expected = 130.848;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 386") {
		constexpr double hysteresis_loss = 4.41752;
		constexpr double eddy_current_loss_coefficient = 0.0188993;
		constexpr double angular_speed = -26.5652;
		constexpr double torque = 31.6163;
		constexpr double expected = -835.978;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 387") {
		constexpr double hysteresis_loss = 3.03853;
		constexpr double eddy_current_loss_coefficient = 0.0434223;
		constexpr double angular_speed = -29.7551;
		constexpr double torque = 94.1509;
		constexpr double expected = -2799.72;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 388") {
		constexpr double hysteresis_loss = 4.04163;
		constexpr double eddy_current_loss_coefficient = 0.0216132;
		constexpr double angular_speed = -7.68933;
		constexpr double torque = 106.651;
		constexpr double expected = -816.198;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 389") {
		constexpr double hysteresis_loss = 2.62952;
		constexpr double eddy_current_loss_coefficient = 0.0187024;
		constexpr double angular_speed = 9.85171;
		constexpr double torque = 119.956;
		constexpr double expected = 1184.59;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 390") {
		constexpr double hysteresis_loss = 4.92845;
		constexpr double eddy_current_loss_coefficient = 0.0377223;
		constexpr double angular_speed = 4.13134;
		constexpr double torque = 79.5977;
		constexpr double expected = 333.929;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 391") {
		constexpr double hysteresis_loss = 4.39493;
		constexpr double eddy_current_loss_coefficient = 0.039453;
		constexpr double angular_speed = -3.83531;
		constexpr double torque = 61.6064;
		constexpr double expected = -232.036;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 392") {
		constexpr double hysteresis_loss = 4.57223;
		constexpr double eddy_current_loss_coefficient = 0.0176494;
		constexpr double angular_speed = 20.6187;
		constexpr double torque = 41.4583;
		constexpr double expected = 859.753;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 393") {
		constexpr double hysteresis_loss = 1.77322;
		constexpr double eddy_current_loss_coefficient = 0.0145611;
		constexpr double angular_speed = -25.227;
		constexpr double torque = 13.8382;
		constexpr double expected = -347.69;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 394") {
		constexpr double hysteresis_loss = 2.91012;
		constexpr double eddy_current_loss_coefficient = 0.00838832;
		constexpr double angular_speed = -20.3859;
		constexpr double torque = 77.6183;
		constexpr double expected = -1579.58;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 395") {
		constexpr double hysteresis_loss = 2.79005;
		constexpr double eddy_current_loss_coefficient = 0.00436938;
		constexpr double angular_speed = 21.6012;
		constexpr double torque = 97.2038;
		constexpr double expected = 2102.6;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 396") {
		constexpr double hysteresis_loss = 4.56513;
		constexpr double eddy_current_loss_coefficient = 0.00971177;
		constexpr double angular_speed = -0.181577;
		constexpr double torque = 101.432;
		constexpr double expected = -13.8543;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 397") {
		constexpr double hysteresis_loss = 3.63307;
		constexpr double eddy_current_loss_coefficient = 0.0352122;
		constexpr double angular_speed = -8.73992;
		constexpr double torque = 92.2082;
		constexpr double expected = -802.566;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 398") {
		constexpr double hysteresis_loss = 4.70766;
		constexpr double eddy_current_loss_coefficient = 0.0291508;
		constexpr double angular_speed = -0.824645;
		constexpr double torque = 20.9506;
		constexpr double expected = -12.5932;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 399") {
		constexpr double hysteresis_loss = 3.24786;
		constexpr double eddy_current_loss_coefficient = 0.0353745;
		constexpr double angular_speed = -11.8929;
		constexpr double torque = 60.5236;
		constexpr double expected = -716.976;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 400") {
		constexpr double hysteresis_loss = 2.72095;
		constexpr double eddy_current_loss_coefficient = 0.0403402;
		constexpr double angular_speed = -1.61931;
		constexpr double torque = 34.0045;
		constexpr double expected = -52.4082;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 401") {
		constexpr double hysteresis_loss = 4.66156;
		constexpr double eddy_current_loss_coefficient = 0.0177696;
		constexpr double angular_speed = 13.3246;
		constexpr double torque = 105.048;
		constexpr double expected = 1404.61;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 402") {
		constexpr double hysteresis_loss = 2.42911;
		constexpr double eddy_current_loss_coefficient = 0.00632898;
		constexpr double angular_speed = 18.8049;
		constexpr double torque = 109.963;
		constexpr double expected = 2070.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 403") {
		constexpr double hysteresis_loss = 4.8502;
		constexpr double eddy_current_loss_coefficient = 0.0172422;
		constexpr double angular_speed = -9.88643;
		constexpr double torque = 104.719;
		constexpr double expected = -1030.62;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 404") {
		constexpr double hysteresis_loss = 3.06681;
		constexpr double eddy_current_loss_coefficient = 0.0173926;
		constexpr double angular_speed = 7.92365;
		constexpr double torque = 30.6297;
		constexpr double expected = 245.904;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 405") {
		constexpr double hysteresis_loss = 4.59268;
		constexpr double eddy_current_loss_coefficient = 0.00691864;
		constexpr double angular_speed = 23.6526;
		constexpr double torque = 25.6186;
		constexpr double expected = 610.701;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 406") {
		constexpr double hysteresis_loss = 4.46579;
		constexpr double eddy_current_loss_coefficient = 0.0275102;
		constexpr double angular_speed = 11.5676;
		constexpr double torque = 73.9484;
		constexpr double expected = 860.188;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 407") {
		constexpr double hysteresis_loss = 3.189;
		constexpr double eddy_current_loss_coefficient = 0.0162131;
		constexpr double angular_speed = -4.00944;
		constexpr double torque = 31.2475;
		constexpr double expected = -122.161;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 408") {
		constexpr double hysteresis_loss = 4.02539;
		constexpr double eddy_current_loss_coefficient = 0.0385421;
		constexpr double angular_speed = -25.2466;
		constexpr double torque = 14.8239;
		constexpr double expected = -371.201;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 409") {
		constexpr double hysteresis_loss = 1.34975;
		constexpr double eddy_current_loss_coefficient = 0.0130956;
		constexpr double angular_speed = 29.6571;
		constexpr double torque = 2.80828;
		constexpr double expected = 85.0237;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 410") {
		constexpr double hysteresis_loss = 1.52063;
		constexpr double eddy_current_loss_coefficient = 0.0376447;
		constexpr double angular_speed = -19.1871;
		constexpr double torque = 83.1807;
		constexpr double expected = -1595.2;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 411") {
		constexpr double hysteresis_loss = 1.37758;
		constexpr double eddy_current_loss_coefficient = 0.0289925;
		constexpr double angular_speed = 17.156;
		constexpr double torque = 9.62762;
		constexpr double expected = 167.046;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 412") {
		constexpr double hysteresis_loss = 4.89699;
		constexpr double eddy_current_loss_coefficient = 0.0345251;
		constexpr double angular_speed = -14.0619;
		constexpr double torque = 32.38;
		constexpr double expected = -450.913;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 413") {
		constexpr double hysteresis_loss = 3.66257;
		constexpr double eddy_current_loss_coefficient = 0.0499618;
		constexpr double angular_speed = 20.4085;
		constexpr double torque = 43.3223;
		constexpr double expected = 888.827;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 414") {
		constexpr double hysteresis_loss = 2.47655;
		constexpr double eddy_current_loss_coefficient = 0.0357776;
		constexpr double angular_speed = -23.769;
		constexpr double torque = 78.1876;
		constexpr double expected = -1856.81;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 415") {
		constexpr double hysteresis_loss = 1.6562;
		constexpr double eddy_current_loss_coefficient = 0.0296922;
		constexpr double angular_speed = 6.63528;
		constexpr double torque = 77.8067;
		constexpr double expected = 518.122;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 416") {
		constexpr double hysteresis_loss = 1.05901;
		constexpr double eddy_current_loss_coefficient = 0.0135877;
		constexpr double angular_speed = -26.0833;
		constexpr double torque = 60.2441;
		constexpr double expected = -1570.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 417") {
		constexpr double hysteresis_loss = 1.9727;
		constexpr double eddy_current_loss_coefficient = 0.0351116;
		constexpr double angular_speed = -16.5505;
		constexpr double torque = 124.714;
		constexpr double expected = -2062.69;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 418") {
		constexpr double hysteresis_loss = 1.60589;
		constexpr double eddy_current_loss_coefficient = 0.0378492;
		constexpr double angular_speed = 8.44181;
		constexpr double torque = 8.05546;
		constexpr double expected = 69.9281;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 419") {
		constexpr double hysteresis_loss = 3.49954;
		constexpr double eddy_current_loss_coefficient = 0.0443338;
		constexpr double angular_speed = -6.01264;
		constexpr double torque = 111.064;
		constexpr double expected = -664.554;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 420") {
		constexpr double hysteresis_loss = 3.53609;
		constexpr double eddy_current_loss_coefficient = 0.0108753;
		constexpr double angular_speed = -10.3109;
		constexpr double torque = 112.261;
		constexpr double expected = -1154.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 421") {
		constexpr double hysteresis_loss = 4.66778;
		constexpr double eddy_current_loss_coefficient = 0.00849598;
		constexpr double angular_speed = -26.5151;
		constexpr double torque = 64.5259;
		constexpr double expected = -1706.47;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 422") {
		constexpr double hysteresis_loss = 1.94112;
		constexpr double eddy_current_loss_coefficient = 0.00794511;
		constexpr double angular_speed = 21.728;
		constexpr double torque = 123.882;
		constexpr double expected = 2693.82;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 423") {
		constexpr double hysteresis_loss = 2.03264;
		constexpr double eddy_current_loss_coefficient = 0.00971447;
		constexpr double angular_speed = -28.1309;
		constexpr double torque = 98.7136;
		constexpr double expected = -2775.14;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 424") {
		constexpr double hysteresis_loss = 1.04223;
		constexpr double eddy_current_loss_coefficient = 0.0447101;
		constexpr double angular_speed = -17.6026;
		constexpr double torque = 36.4207;
		constexpr double expected = -640.844;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 425") {
		constexpr double hysteresis_loss = 2.69097;
		constexpr double eddy_current_loss_coefficient = 0.0368221;
		constexpr double angular_speed = -17.2891;
		constexpr double torque = 79.6599;
		constexpr double expected = -1375.2;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 426") {
		constexpr double hysteresis_loss = 4.98536;
		constexpr double eddy_current_loss_coefficient = 0.00860114;
		constexpr double angular_speed = 3.67422;
		constexpr double torque = 86.9956;
		constexpr double expected = 324.658;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 427") {
		constexpr double hysteresis_loss = 2.47453;
		constexpr double eddy_current_loss_coefficient = 0.0398956;
		constexpr double angular_speed = -22.2232;
		constexpr double torque = 55.7997;
		constexpr double expected = -1238.46;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 428") {
		constexpr double hysteresis_loss = 4.18058;
		constexpr double eddy_current_loss_coefficient = 0.0441511;
		constexpr double angular_speed = 21.0113;
		constexpr double torque = 37.1533;
		constexpr double expected = 785.747;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 429") {
		constexpr double hysteresis_loss = 2.83647;
		constexpr double eddy_current_loss_coefficient = 0.0478436;
		constexpr double angular_speed = 8.12022;
		constexpr double torque = 80.3737;
		constexpr double expected = 655.877;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 430") {
		constexpr double hysteresis_loss = 2.63219;
		constexpr double eddy_current_loss_coefficient = 0.0206466;
		constexpr double angular_speed = -3.44925;
		constexpr double torque = 54.1981;
		constexpr double expected = -184.382;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 431") {
		constexpr double hysteresis_loss = 1.2076;
		constexpr double eddy_current_loss_coefficient = 0.0310471;
		constexpr double angular_speed = -19.546;
		constexpr double torque = 18.0865;
		constexpr double expected = -352.917;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 432") {
		constexpr double hysteresis_loss = 4.0841;
		constexpr double eddy_current_loss_coefficient = 0.0454147;
		constexpr double angular_speed = 24.4812;
		constexpr double torque = 72.7764;
		constexpr double expected = 1786.85;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 433") {
		constexpr double hysteresis_loss = 4.72752;
		constexpr double eddy_current_loss_coefficient = 0.0437645;
		constexpr double angular_speed = -12.4147;
		constexpr double torque = 8.75231;
		constexpr double expected = -104.473;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 434") {
		constexpr double hysteresis_loss = 3.58707;
		constexpr double eddy_current_loss_coefficient = 0.00347368;
		constexpr double angular_speed = 17.9972;
		constexpr double torque = 103.481;
		constexpr double expected = 1866.02;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 435") {
		constexpr double hysteresis_loss = 2.64116;
		constexpr double eddy_current_loss_coefficient = 0.0328094;
		constexpr double angular_speed = 7.06325;
		constexpr double torque = 113.715;
		constexpr double expected = 806.069;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 436") {
		constexpr double hysteresis_loss = 4.02125;
		constexpr double eddy_current_loss_coefficient = 0.0438359;
		constexpr double angular_speed = 22.5416;
		constexpr double torque = 2.60586;
		constexpr double expected = 63.7497;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 437") {
		constexpr double hysteresis_loss = 2.8535;
		constexpr double eddy_current_loss_coefficient = 0.0257934;
		constexpr double angular_speed = -22.5047;
		constexpr double torque = 81.7335;
		constexpr double expected = -1837.12;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 438") {
		constexpr double hysteresis_loss = 3.73022;
		constexpr double eddy_current_loss_coefficient = 0.0444413;
		constexpr double angular_speed = 7.36599;
		constexpr double torque = 113.868;
		constexpr double expected = 842.808;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 439") {
		constexpr double hysteresis_loss = 4.54491;
		constexpr double eddy_current_loss_coefficient = 0.00669336;
		constexpr double angular_speed = -24.4198;
		constexpr double torque = 25.4165;
		constexpr double expected = -616.283;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 440") {
		constexpr double hysteresis_loss = 4.0352;
		constexpr double eddy_current_loss_coefficient = 0.0272976;
		constexpr double angular_speed = -19.7123;
		constexpr double torque = 85.6479;
		constexpr double expected = -1684.82;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 441") {
		constexpr double hysteresis_loss = 2.62416;
		constexpr double eddy_current_loss_coefficient = 0.0228846;
		constexpr double angular_speed = 2.02538;
		constexpr double torque = 1.96601;
		constexpr double expected = 6.65243;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 442") {
		constexpr double hysteresis_loss = 4.16312;
		constexpr double eddy_current_loss_coefficient = 0.00618968;
		constexpr double angular_speed = -24.5728;
		constexpr double torque = 9.43451;
		constexpr double expected = -227.821;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 443") {
		constexpr double hysteresis_loss = 1.63782;
		constexpr double eddy_current_loss_coefficient = 0.0321256;
		constexpr double angular_speed = -1.27847;
		constexpr double torque = 26.6258;
		constexpr double expected = -32.4437;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 444") {
		constexpr double hysteresis_loss = 4.61924;
		constexpr double eddy_current_loss_coefficient = 0.0166909;
		constexpr double angular_speed = -27.887;
		constexpr double torque = 78.8385;
		constexpr double expected = -2194.42;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 445") {
		constexpr double hysteresis_loss = 1.7577;
		constexpr double eddy_current_loss_coefficient = 0.0167592;
		constexpr double angular_speed = -19.7722;
		constexpr double torque = 36.7698;
		constexpr double expected = -725.594;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 446") {
		constexpr double hysteresis_loss = 2.02215;
		constexpr double eddy_current_loss_coefficient = 0.0236974;
		constexpr double angular_speed = -7.33342;
		constexpr double torque = 77.8747;
		constexpr double expected = -569.239;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 447") {
		constexpr double hysteresis_loss = 1.86756;
		constexpr double eddy_current_loss_coefficient = 0.00434999;
		constexpr double angular_speed = 23.0768;
		constexpr double torque = 43.0944;
		constexpr double expected = 996.45;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 448") {
		constexpr double hysteresis_loss = 3.77125;
		constexpr double eddy_current_loss_coefficient = 0.0453054;
		constexpr double angular_speed = -17.8361;
		constexpr double torque = 108.705;
		constexpr double expected = -1935.91;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 449") {
		constexpr double hysteresis_loss = 1.7523;
		constexpr double eddy_current_loss_coefficient = 0.0170371;
		constexpr double angular_speed = -6.93647;
		constexpr double torque = 7.50163;
		constexpr double expected = -50.4007;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 450") {
		constexpr double hysteresis_loss = 4.37606;
		constexpr double eddy_current_loss_coefficient = 0.0103328;
		constexpr double angular_speed = 18.4706;
		constexpr double torque = 16.0923;
		constexpr double expected = 301.801;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 451") {
		constexpr double hysteresis_loss = 2.28671;
		constexpr double eddy_current_loss_coefficient = 0.0490734;
		constexpr double angular_speed = 24.333;
		constexpr double torque = 69.4905;
		constexpr double expected = 1694.39;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 452") {
		constexpr double hysteresis_loss = 3.72071;
		constexpr double eddy_current_loss_coefficient = 0.033383;
		constexpr double angular_speed = 9.56067;
		constexpr double torque = 56.723;
		constexpr double expected = 546.35;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 453") {
		constexpr double hysteresis_loss = 3.28943;
		constexpr double eddy_current_loss_coefficient = 0.0113319;
		constexpr double angular_speed = 26.257;
		constexpr double torque = 9.59027;
		constexpr double expected = 255.399;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 454") {
		constexpr double hysteresis_loss = 3.14966;
		constexpr double eddy_current_loss_coefficient = 0.0297535;
		constexpr double angular_speed = 22.2225;
		constexpr double torque = 69.814;
		constexpr double expected = 1555.25;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 455") {
		constexpr double hysteresis_loss = 2.86045;
		constexpr double eddy_current_loss_coefficient = 0.0123267;
		constexpr double angular_speed = 10.5839;
		constexpr double torque = 48.6052;
		constexpr double expected = 517.423;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 456") {
		constexpr double hysteresis_loss = 1.18032;
		constexpr double eddy_current_loss_coefficient = 0.0172031;
		constexpr double angular_speed = -3.88392;
		constexpr double torque = 87.7471;
		constexpr double expected = -339.689;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 457") {
		constexpr double hysteresis_loss = 2.36071;
		constexpr double eddy_current_loss_coefficient = 0.00723372;
		constexpr double angular_speed = 20.726;
		constexpr double torque = 36.605;
		constexpr double expected = 761.186;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 458") {
		constexpr double hysteresis_loss = 3.16761;
		constexpr double eddy_current_loss_coefficient = 0.0489789;
		constexpr double angular_speed = -14.4197;
		constexpr double torque = 28.3003;
		constexpr double expected = -405.619;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 459") {
		constexpr double hysteresis_loss = 1.35708;
		constexpr double eddy_current_loss_coefficient = 0.0252425;
		constexpr double angular_speed = 1.51897;
		constexpr double torque = 3.76594;
		constexpr double expected = 7.11578;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 460") {
		constexpr double hysteresis_loss = 2.83202;
		constexpr double eddy_current_loss_coefficient = 0.00940509;
		constexpr double angular_speed = -29.4891;
		constexpr double torque = 46.2414;
		constexpr double expected = -1361.06;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 461") {
		constexpr double hysteresis_loss = 1.9218;
		constexpr double eddy_current_loss_coefficient = 0.0178074;
		constexpr double angular_speed = 11.9391;
		constexpr double torque = 93.1687;
		constexpr double expected = 1114.49;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 462") {
		constexpr double hysteresis_loss = 1.07437;
		constexpr double eddy_current_loss_coefficient = 0.0133707;
		constexpr double angular_speed = 15.6701;
		constexpr double torque = 118.506;
		constexpr double expected = 1858.29;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 463") {
		constexpr double hysteresis_loss = 2.09221;
		constexpr double eddy_current_loss_coefficient = 0.0192392;
		constexpr double angular_speed = -25.633;
		constexpr double torque = 28.9893;
		constexpr double expected = -741.485;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 464") {
		constexpr double hysteresis_loss = 3.38081;
		constexpr double eddy_current_loss_coefficient = 0.0400829;
		constexpr double angular_speed = 20.3205;
		constexpr double torque = 101.187;
		constexpr double expected = 2060.37;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 465") {
		constexpr double hysteresis_loss = 3.87987;
		constexpr double eddy_current_loss_coefficient = 0.0135393;
		constexpr double angular_speed = -13.9363;
		constexpr double torque = 104.362;
		constexpr double expected = -1450.72;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 466") {
		constexpr double hysteresis_loss = 1.68915;
		constexpr double eddy_current_loss_coefficient = 0.00164173;
		constexpr double angular_speed = -6.80526;
		constexpr double torque = 88.4922;
		constexpr double expected = -600.534;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 467") {
		constexpr double hysteresis_loss = 4.4535;
		constexpr double eddy_current_loss_coefficient = 0.00455682;
		constexpr double angular_speed = -19.7907;
		constexpr double torque = 25.4917;
		constexpr double expected = -500.137;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 468") {
		constexpr double hysteresis_loss = 3.57245;
		constexpr double eddy_current_loss_coefficient = 0.0371743;
		constexpr double angular_speed = 29.1708;
		constexpr double torque = 121.66;
		constexpr double expected = 3553.57;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 469") {
		constexpr double hysteresis_loss = 4.29475;
		constexpr double eddy_current_loss_coefficient = 0.0300909;
		constexpr double angular_speed = 8.29976;
		constexpr double torque = 109.943;
		constexpr double expected = 917.044;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 470") {
		constexpr double hysteresis_loss = 2.51603;
		constexpr double eddy_current_loss_coefficient = 0.0130532;
		constexpr double angular_speed = -14.5573;
		constexpr double torque = 45.9318;
		constexpr double expected = -666.316;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 471") {
		constexpr double hysteresis_loss = 3.02296;
		constexpr double eddy_current_loss_coefficient = 0.0284872;
		constexpr double angular_speed = 17.0184;
		constexpr double torque = 31.4209;
		constexpr double expected = 538.241;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 472") {
		constexpr double hysteresis_loss = 3.40656;
		constexpr double eddy_current_loss_coefficient = 0.00214903;
		constexpr double angular_speed = 8.28995;
		constexpr double torque = 96.599;
		constexpr double expected = 804.225;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 473") {
		constexpr double hysteresis_loss = 2.80833;
		constexpr double eddy_current_loss_coefficient = 0.019734;
		constexpr double angular_speed = -10.913;
		constexpr double torque = 44.8353;
		constexpr double expected = -486.695;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 474") {
		constexpr double hysteresis_loss = 1.47137;
		constexpr double eddy_current_loss_coefficient = 0.0310193;
		constexpr double angular_speed = -26.6324;
		constexpr double torque = 69.4005;
		constexpr double expected = -1847.66;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 475") {
		constexpr double hysteresis_loss = 1.76129;
		constexpr double eddy_current_loss_coefficient = 0.0426562;
		constexpr double angular_speed = 26.0323;
		constexpr double torque = 38.9593;
		constexpr double expected = 1017.07;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 476") {
		constexpr double hysteresis_loss = 4.42986;
		constexpr double eddy_current_loss_coefficient = 0.0210341;
		constexpr double angular_speed = 13.4399;
		constexpr double torque = 117.595;
		constexpr double expected = 1585.18;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 477") {
		constexpr double hysteresis_loss = 4.39736;
		constexpr double eddy_current_loss_coefficient = 0.0445262;
		constexpr double angular_speed = 16.3829;
		constexpr double torque = 120.239;
		constexpr double expected = 1974.99;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 478") {
		constexpr double hysteresis_loss = 2.66248;
		constexpr double eddy_current_loss_coefficient = 0.00130102;
		constexpr double angular_speed = -1.05916;
		constexpr double torque = 85.8359;
		constexpr double expected = -88.2528;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 479") {
		constexpr double hysteresis_loss = 3.09749;
		constexpr double eddy_current_loss_coefficient = 0.0470871;
		constexpr double angular_speed = 29.3532;
		constexpr double torque = 69.3863;
		constexpr double expected = 2041.19;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 480") {
		constexpr double hysteresis_loss = 3.54379;
		constexpr double eddy_current_loss_coefficient = 0.0378883;
		constexpr double angular_speed = 2.58167;
		constexpr double torque = 66.0125;
		constexpr double expected = 174.064;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 481") {
		constexpr double hysteresis_loss = 1.3568;
		constexpr double eddy_current_loss_coefficient = 0.0323393;
		constexpr double angular_speed = -12.8148;
		constexpr double torque = 1.37021;
		constexpr double expected = -16.6165;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 482") {
		constexpr double hysteresis_loss = 4.94273;
		constexpr double eddy_current_loss_coefficient = 0.00333827;
		constexpr double angular_speed = 15.487;
		constexpr double torque = 82.3385;
		constexpr double expected = 1280.17;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 483") {
		constexpr double hysteresis_loss = 1.7737;
		constexpr double eddy_current_loss_coefficient = 0.0390647;
		constexpr double angular_speed = 3.25144;
		constexpr double torque = 26.2387;
		constexpr double expected = 87.2141;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 484") {
		constexpr double hysteresis_loss = 2.09455;
		constexpr double eddy_current_loss_coefficient = 0.0280265;
		constexpr double angular_speed = 8.96903;
		constexpr double torque = 73.7148;
		constexpr double expected = 663.497;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 485") {
		constexpr double hysteresis_loss = 4.22634;
		constexpr double eddy_current_loss_coefficient = 0.00889614;
		constexpr double angular_speed = 4.2951;
		constexpr double torque = 108.41;
		constexpr double expected = 469.896;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 486") {
		constexpr double hysteresis_loss = 3.64141;
		constexpr double eddy_current_loss_coefficient = 0.0148342;
		constexpr double angular_speed = 25.7134;
		constexpr double torque = 74.6073;
		constexpr double expected = 1922.43;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 487") {
		constexpr double hysteresis_loss = 4.23268;
		constexpr double eddy_current_loss_coefficient = 0.0358619;
		constexpr double angular_speed = 18.7123;
		constexpr double torque = 89.0435;
		constexpr double expected = 1671.12;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 488") {
		constexpr double hysteresis_loss = 2.29408;
		constexpr double eddy_current_loss_coefficient = 0.0264011;
		constexpr double angular_speed = 12.5263;
		constexpr double torque = 100.773;
		constexpr double expected = 1264.94;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 489") {
		constexpr double hysteresis_loss = 1.72931;
		constexpr double eddy_current_loss_coefficient = 0.0343481;
		constexpr double angular_speed = 3.91357;
		constexpr double torque = 61.065;
		constexpr double expected = 240.846;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 490") {
		constexpr double hysteresis_loss = 4.65686;
		constexpr double eddy_current_loss_coefficient = 0.0126019;
		constexpr double angular_speed = 19.0339;
		constexpr double torque = 107.443;
		constexpr double expected = 2049.95;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 491") {
		constexpr double hysteresis_loss = 3.8645;
		constexpr double eddy_current_loss_coefficient = 0.0420447;
		constexpr double angular_speed = -29.0058;
		constexpr double torque = 101.66;
		constexpr double expected = -2946.09;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 492") {
		constexpr double hysteresis_loss = 1.42187;
		constexpr double eddy_current_loss_coefficient = 0.0360891;
		constexpr double angular_speed = 4.01241;
		constexpr double torque = 100.436;
		constexpr double expected = 404.557;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 493") {
		constexpr double hysteresis_loss = 4.7505;
		constexpr double eddy_current_loss_coefficient = 0.033645;
		constexpr double angular_speed = 5.54114;
		constexpr double torque = 56.5793;
		constexpr double expected = 318.45;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 494") {
		constexpr double hysteresis_loss = 4.85399;
		constexpr double eddy_current_loss_coefficient = 0.0402393;
		constexpr double angular_speed = -26.6009;
		constexpr double torque = 60.5356;
		constexpr double expected = -1606.52;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 495") {
		constexpr double hysteresis_loss = 3.7136;
		constexpr double eddy_current_loss_coefficient = 0.00180902;
		constexpr double angular_speed = 17.3794;
		constexpr double torque = 8.81878;
		constexpr double expected = 157.01;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 496") {
		constexpr double hysteresis_loss = 3.59017;
		constexpr double eddy_current_loss_coefficient = 0.0349853;
		constexpr double angular_speed = 24.8128;
		constexpr double torque = 6.04188;
		constexpr double expected = 154.374;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 497") {
		constexpr double hysteresis_loss = 4.50131;
		constexpr double eddy_current_loss_coefficient = 0.00290389;
		constexpr double angular_speed = 10.9025;
		constexpr double torque = 91.4144;
		constexpr double expected = 1001.18;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 498") {
		constexpr double hysteresis_loss = 4.76063;
		constexpr double eddy_current_loss_coefficient = 0.0126767;
		constexpr double angular_speed = -1.23862;
		constexpr double torque = 80.5918;
		constexpr double expected = -95.0778;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
	SECTION("Random Test 499") {
		constexpr double hysteresis_loss = 1.56426;
		constexpr double eddy_current_loss_coefficient = 0.0314632;
		constexpr double angular_speed = -18.3916;
		constexpr double torque = 111.848;
		constexpr double expected = -2056.08;
		const auto motor = Motor(hysteresis_loss, eddy_current_loss_coefficient);
		const auto result = motor.power_consumed(angular_speed, torque);
		REQUIRE_THAT(result, WithinRel(expected, EPSILON));
	}
}
