#ifndef MINISIM_SOLARCAR_H
#define MINISIM_SOLARCAR_H

#include <string>

#include "ConfigFile/ConfigFile.h"

// Car Components
#include "SolarCar/Aerobody/Aerobody.h"
#include "SolarCar/Array/Array.h"
#include "SolarCar/Battery/Battery.h"
#include "SolarCar/Motor/Motor.h"
#include "SolarCar/Tire/Tire.h"

class SolarCar {
   public:
	Aerobody aerobody;
	Array array;
	Battery battery;
	Motor motor;
	Tire tire;
	double mass;
	double wheel_radius;

	SolarCar(const Aerobody& aerobody, const Array& array, const Battery& battery, const Motor& motor, const Tire& tire,
		const double mass, const double wheel_radius)
		: aerobody(aerobody),
		  array(array),
		  battery(battery),
		  motor(motor),
		  tire(tire),
		  mass(mass),
		  wheel_radius(wheel_radius) {}
	explicit SolarCar(const ConfigFile& car_config);
};

#endif  // MINISIM_SOLARCAR_H