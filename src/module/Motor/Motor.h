#ifndef MINISIM_MOTOR_H
#define MINISIM_MOTOR_H

class Motor {
   public:
	Motor(double hysteresis_loss, double eddy_current_loss_coefficient)
		: hysteresis_loss(hysteresis_loss),
		  eddy_current_loss_coefficient(eddy_current_loss_coefficient) {
	}

	/// @brief Calculates the power the motor consumed to rotate at
	//// @p angular_speed with @p torque.
    /// @note For additional losses, only consider Hysteresis and Eddy Current  
    /// for this model.
	///
	/// @param angular_speed (rad/s) the angular speed the motor is rotating at.
	/// @param torque (Nm) the torque that the motor is operating with.
	/// @return power consumed (W)
    ///
	/// @note negative torque means regenerative braking.
	double power_consumed(double angular_speed, double torque) const;

   private:
	/// @brief (W) the losses associated with the hysteresis of the motor.
	///
	/// @note this can be determined experimentally with enough data.
	double hysteresis_loss;

	/// @brief (unitless) the coefficient used to determine the losses
	/// associated with the eddy currents in the motor.
	///
	/// @note this can be determined experimentally with enough data.
	double eddy_current_loss_coefficient;
};

#endif  // MINISIM_MOTOR_H
