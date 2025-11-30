#ifndef MINISIM_PARSING_H
#define MINISIM_PARSING_H

#include <string>

#include "alglib/interpolation.h"

/**
 * Get a cubic spline from a CSV file containing the given fieldnames.
 * @param x_fieldname is the independent variable
 * @param y_fieldname is the dependent variable
 */
using Spline1DBuilder = void (*)(
	const alglib::real_1d_array&, const alglib::real_1d_array&, alglib::spline1dinterpolant&);

void build_linear_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline);

void build_cubic_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline);

void build_akima_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline);

/// Get a 1D spline from a CSV file containing the given fieldnames.
/// @param x_fieldname is the independent variable
/// @param y_fieldname is the independent variable
alglib::spline1dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, Spline1DBuilder builder_fun);

/// Get a 2D spline from a CSV file containing the given fieldnames.
/// @param x_fieldname is the independent variable
/// @param y_fieldname is the independent variable
/// @param z_fieldname is the dependent variable
/// @param dim_x is the number of unique values of x_fieldname
/// @param dim_y is the number of unique values of y_fieldname
alglib::spline2dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, const std::string& z_fieldname, size_t dim_x, size_t dim_y);

/// Get a 3D spline from a CSV file containing the given fieldnames.
/// @param x_fieldname is the independent variable
/// @param y_fieldname is the independent variable
/// @param z_fieldname is the independent variable
/// @param w_fieldname is the dependent variable
/// @param dim_x is the number of unique values of x_fieldname
/// @param dim_y is the number of unique values of y_fieldname
/// @param dim_z is the number of unique values of z_fieldname
/// @return a 3D spline
alglib::spline3dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, const std::string& z_fieldname, const std::string& w_fieldname, size_t dim_x,
	size_t dim_y, size_t dim_z);

#endif  // MINISIM_PARSING_H