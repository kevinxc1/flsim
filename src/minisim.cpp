#include <getopt.h>

#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "ConfigFile/ConfigFile.h"
#include "Optimizer/Optimizer.h"
#include "RaceConfig/RaceSchedule/RaceSchedule.h"
#include "RaceConfig/Route/Route.h"
#include "RaceConfig/Weather/Weather.h"
#include "RaceConfig/WeatherStations/WeatherStations.h"
#include "SolarCar/SolarCar.h"
#include "Tools/Conversions.h"

namespace {
	struct CommandLine {
		std::string car_file;
		std::string weather_file;
		std::string weather_stations_file;
		std::string route_file;
		std::string schedule_file;
		std::string optimizer_type;
	};

	void print_help() {
		std::cout << "Usage: Simulator -o <optimizer-type> -c <car.toml> -w <weather.csv> -r <route.csv> -t "
					 "<weather_stations.csv> -s <schedule.toml>\n\n"
				  << "Run the simulator to optimize your car!\n\n"
				  << "Options:\n"
				  << "  -h, --help        display this help and exit\n"
				  << "  -o, --optimizer   the optimizer to use (e.g. linear, binary)\n"
				  << "  -c, --car         the car config file to use (TOML)\n"
				  << "  -w, --weather     the weather file to use (CSV)\n"
				  << "  -r, --route       the route file to use (CSV)\n"
				  << "  -t, --stations    the weather stations being used (CSV)\n"
				  << "  -s, --schedule    the schedule file to use (TOML)\n";
	}

	CommandLine read_args(const int argc, char** argv) {
		opterr = 0;
		int choice = 0;
		int index = 0;

		 
		static struct option long_options[] = {
			{"car",      required_argument, nullptr, 'c'},
			{"weather",  required_argument, nullptr, 'w'},
			{"route",    required_argument, nullptr, 'r'},
			{"schedule", required_argument, nullptr, 's'},
			{"stations", required_argument, nullptr, 't'},
			{"help",     no_argument,       nullptr, 'h'},
			{nullptr,    0,                 nullptr, 0  },
		};

		CommandLine config = {};

		enum Params {
			 
			Car                  = 0b000001,
			Weather              = 0b000010,
			Route                = 0b000100,
			Schedule             = 0b001000,
			WeatherStation       = 0b010000,
			Optimizer            = 0b100000,
			All                  = 0b111111,
			 
		};

		uint8_t params_received = 0;

		 
		while ((choice = getopt_long(argc, argv, "hc:w:r:s:t:o:", long_options, &index)) != -1) {
			switch (choice) {
				case 'h': {
					print_help();
					exit(0);   
				}
				case 'c': {
					config.car_file = std::string(optarg);
					std::cout << "[CONFIG] Car File: " << config.car_file << "\n";
					params_received |= Params::Car;
					break;
				}
				case 'w': {
					config.weather_file = std::string(optarg);
					std::cout << "[CONFIG] Weather File: " << config.weather_file << "\n";
					params_received |= Params::Weather;
					break;
				}
				case 'r': {
					config.route_file = std::string(optarg);
					std::cout << "[CONFIG] Route File: " << config.route_file << "\n";
					params_received |= Params::Route;
					break;
				}
				case 's': {
					config.schedule_file = std::string(optarg);
					std::cout << "[CONFIG] Schedule File: " << config.schedule_file << "\n";
					params_received |= Params::Schedule;
					break;
				}
				case 't': {
					config.weather_stations_file = std::string(optarg);
					std::cout << "[CONFIG] Weather Stations File: " << config.weather_stations_file << "\n";
					params_received |= Params::WeatherStation;
					break;
				}
				case 'o': {
					config.optimizer_type = std::string(optarg);
					std::cout << "[CONFIG] Optimizer Type: " << config.optimizer_type << "\n";
					params_received |= Params::Optimizer;
					break;
				}
				default: {
					std::cerr << "Invalid option: " << static_cast<char>(choice) << "\n\n";
					print_help();
					exit(1);   
				}
			}
		}
		if (params_received != Params::All) {
			std::cerr << "\n[ERROR] Missing Option: Not all required options were passed.\n\n";
			print_help();
			exit(2);   
		}
		std::cout << std::flush;
		return config;
	}
}   

int main(int argc, char** argv) {
	const auto config = read_args(argc, argv);

	ConfigFile car_config;
	{   
		const auto car_config_opt = ConfigFile::from_path(config.car_file);
		if (!car_config_opt.has_value()) {
			std::cerr << "[ERROR] Car Config is Invalid\n";
			exit(2);   
		}
		car_config = car_config_opt.value();
	}
	ConfigFile schedule_config;
	{   
		const auto schedule_config_opt = ConfigFile::from_path(config.schedule_file);
		if (!schedule_config_opt.has_value()) {
			std::cerr << "[ERROR] Schedule is Invalid\n";
			exit(2);   
		}
		schedule_config = schedule_config_opt.value();
	}

	const auto solarcar = SolarCar(car_config);
	const auto weather_stations = WeatherStations(config.weather_stations_file);
	const auto weather = Weather(config.weather_file, weather_stations);
	const auto route = Route(config.route_file, weather_stations);
	const auto schedule = RaceSchedule(schedule_config);

	const std::unique_ptr<const Optimizer> optimizer =
		Optimizer::create_optimizer(config.optimizer_type, solarcar, weather, route, schedule);

	const auto solution_opt = optimizer->optimize_race();
	constexpr int precision = 5;
	std::cout << std::fixed << std::setprecision(precision) << std::setfill('0');

	if (!solution_opt.has_value()) {
		std::cout << "\n"   
				  << "[OUTPUT] The Car was not able to finish the race.\n\n";
		return 0;
	};

	const auto solution = solution_opt.value();
	std::cout << "\n"   
			  << "[OUTPUT] Race Time: " << solution.racetime << " seconds = " << seconds_to_hours(solution.racetime)
			  << " hours\n"
			  << "[OUTPUT] Optimal Speed: " << solution.speed << " mps = " << mps_to_kph(solution.speed) << " kph\n";
}
