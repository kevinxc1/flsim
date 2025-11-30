#ifndef MINISIM_ARRAY_H
#define MINISIM_ARRAY_H

class Array {
   public:
	/// @param array_area (m^2) the area of the solar array
	/// @param array_efficiency (%) the efficiency of the solar cells we are using
	Array(double array_area, double array_efficiency)
		: array_area(array_area), array_efficiency(array_efficiency){};

	/// @brief calculates the amount of power that the solar array is bringing in
	///
	/// @param irradiance (W/m^2) the irradiance that the array is experiencing
    /// @return power received (W)
	double power_in(double irradiance) const;

   private:
	/// @brief (m^2) the exposed surface area of the solar array
	double array_area;

	/// @brief (%) the efficiency of the solar cells we are using
	double array_efficiency;
};

#endif
