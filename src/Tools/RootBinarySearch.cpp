#include "RootBinarySearch.h"

#include <iostream>

double binary_search(const std::function<double(double)>& objective_function, double initial_guess,
	double error_threshold, double initial_bounds_right) {
	double cur_x = initial_guess;
	double cur_y = objective_function(cur_x);

	 
	double bounds_left = 0;
	double bounds_right = initial_bounds_right;
	if (cur_y > 0) {
		bounds_right = cur_x;
	} else if (cur_y < 0) {
		bounds_left = cur_x;
	}

	 
	while (std::abs(cur_y) > error_threshold) {
		cur_x = (bounds_left + bounds_right) / 2;
		cur_y = objective_function(cur_x);

		if (cur_y > 0) {
			bounds_right = cur_x;
		} else if (cur_y < 0) {
			bounds_left = cur_x;
		}
	}

	return cur_x;
}
