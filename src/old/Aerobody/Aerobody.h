#ifndef MINISIM_AEROBODY_H
#define MINISIM_AEROBODY_H

#include "VelocityVector.hpp"

class Aerobody {
   public:
	Aerobody(double drag_coefficient, double frontal_area)
		: drag_coefficient(drag_coefficient), frontal_area(frontal_area) {};

	/// @brief Calculates the apparent wind speed and yaw
	///
	/// @param reported_wind the wind as reported by a weather station
	/// @note weather stations report where wind is coming from
	/// @note we define "True Wind" as where the wind is going to
	/// @param car_velocity (m/s) the velocity of the car
	///
	/// The apparent wind is the motion of the air around the car, relative to the car--assuming the
	/// frame of reference is the car, so it would include the "wind" created by the car's motion,
	/// on top of the wind that the weather station reports. 
    ///
	/// @return an apparent wind vector
	static ApparentWindVector get_wind(const VelocityVector& reported_wind, const VelocityVector& car_velocity);

	/// @brief Gets the drag on the aerobody at a given apparent wind vector
	///
	/// @param apparent_wind an apparent wind vector with yaw and speed
	/// @param air_density (kg/m^3) The air density
	///
	/// @return the drag force, in Newtons.
	double aerodynamic_drag(const ApparentWindVector& apparent_wind, double air_density) const;

   private:
	/// @brief the coefficient of drag
	///
	/// @note this variable encodes a lot of information, and typically is not static. As we
	/// progress to more realistic models of drag, we'll start calculating the coefficient ourselves
	/// using other information we have about the car, including the direction of the apparent wind,
	/// and Reynolds number (related to speed)
	double drag_coefficient;

	/// @brief the frontal area of the car
	///
	/// @note based on things like steer angle and other non-static parts of the car, this variable
	/// may also be non-constant. However, for simplification, we will assume that it is for now.
	double frontal_area;
};

#endif  // MINISIM_AEROBODY_H
