#ifndef MINISIM_NEWTONRAPHSON_H
#define MINISIM_NEWTONRAPHSON_H

#include <functional>

/// @brief Finds a root of a function using binary search
/// @param objective_function the function to find a root of, must have a single root within [0, initial_bounds_right]
/// @param initial_guess an initial guess for the root of the function, used instead of the midpoint of bounds on the
/// first iteration
/// @param error_threshold the maximum acceptable error, the search will continue until below this
/// @param initial_bounds_right the maximum value that the root could be at, used as an initial upper bound
double binary_search(const std::function<double(double)>& objective_function, double initial_guess,
	double error_threshold, double initial_bounds_right);

#endif  // MINISIM_NEWTONRAPHSON_H
