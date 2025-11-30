#include "Parsing.h"

#include <algorithm>
#include <exception>
#include <fstream>
#include <iostream>

#include "csv/csv.h"

using std::ifstream;
using std::string;
using std::vector;

void build_linear_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline) {
	alglib::spline1dbuildlinear(arr1, arr2, spline);
}

void build_cubic_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline) {
	alglib::spline1dbuildcubic(arr1, arr2, spline);
}

void build_akima_wrapper(
	const alglib::real_1d_array& arr1, const alglib::real_1d_array& arr2, alglib::spline1dinterpolant& spline) {
	alglib::spline1dbuildakima(arr1, arr2, spline);
}

alglib::spline1dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, Spline1DBuilder builder_fun) {
	 

	 
	std::ifstream file(filename);
	assert(file.good());

	 
	io::CSVReader<2> in(filename);
	in.read_header(io::ignore_extra_column, x_fieldname, y_fieldname);

	 
	assert(in.has_column(x_fieldname));
	assert(in.has_column(y_fieldname));

	 
	double x, y;
	std::vector<double> x_vec, y_vec;
	while (in.read_row(x, y)) {
		x_vec.push_back(x);
		y_vec.push_back(y);

		 
		assert(!(isnan(x) || isnan(y)));
		assert(!(isinf(x) || isinf(y)));
	}

	 
	struct XY {
		double x;
		double y;
	};

	std::vector<XY> xy_vec;
	xy_vec.reserve(x_vec.size());
	for (size_t i = 0; i < x_vec.size(); ++i) {
		xy_vec.push_back({x_vec[i], y_vec[i]});
	}

	auto comp = [](const auto& lhs, const auto& rhs) { return lhs.x < rhs.x; };
	std::sort(xy_vec.begin(), xy_vec.end(), comp);

	for (size_t i = 0; i < xy_vec.size(); ++i) {
		x_vec[i] = xy_vec[i].x;
		y_vec[i] = xy_vec[i].y;
	}

	 
	alglib::real_1d_array x_arr;
	alglib::real_1d_array y_arr;
	x_arr.setcontent(x_vec.size(), x_vec.data());
	y_arr.setcontent(y_vec.size(), y_vec.data());

	 
	alglib::spline1dinterpolant spline;
	try {
		builder_fun(x_arr, y_arr, spline);
	} catch (alglib::ap_error e) {
		throw std::exception();
	}
	return spline;
}

alglib::spline2dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, const std::string& z_fieldname, size_t dim_x, size_t dim_y) {
	 
	std::ifstream file(filename);
	assert(file.good());

	 
	io::CSVReader<3> in(filename);
	in.read_header(io::ignore_extra_column, x_fieldname, y_fieldname, z_fieldname);

	vector<double> x_vec;
	vector<double> y_vec;
	vector<double> z_vec;
	x_vec.reserve(dim_x * dim_y);
	y_vec.reserve(dim_x * dim_y);
	z_vec.reserve(dim_x * dim_y);

	 
	assert(in.has_column(x_fieldname));
	assert(in.has_column(y_fieldname));
	assert(in.has_column(z_fieldname));

	 
	double x, y, z;
	while (in.read_row(x, y, z)) {
		x_vec.push_back(x);
		y_vec.push_back(y);
		z_vec.push_back(z);

		 
		assert(!(isnan(x) || isnan(y) || isnan(z)));
		assert(!(isinf(x) || isinf(y) || isinf(z)));
	}
	 
	assert(x_vec.size() == dim_x * dim_y);
	 
	struct XYZ {
		double x;
		double y;
		double z;
	};

	std::vector<XYZ> xyz_vec;
	xyz_vec.reserve(x_vec.size());
	for (size_t i = 0; i < x_vec.size(); ++i) {
		xyz_vec.push_back({x_vec[i], y_vec[i], z_vec[i]});
	}

	auto comp = [](const auto& lhs, const auto& rhs) {
		if (lhs.y < rhs.y) {
			return true;
		} else if (lhs.y > rhs.y) {
			return false;
		}
		return lhs.x < rhs.x;
	};

	std::sort(xyz_vec.begin(), xyz_vec.end(), comp);

	for (size_t i = 0; i < xyz_vec.size(); ++i) {
		x_vec[i] = xyz_vec[i].x;
		y_vec[i] = xyz_vec[i].y;
		z_vec[i] = xyz_vec[i].z;
	}

	 
	alglib::real_1d_array x_arr;
	alglib::real_1d_array y_arr;
	alglib::real_1d_array z_arr;
	x_arr.setlength(dim_x);
	y_arr.setlength(dim_y);
	z_arr.setcontent(dim_x * dim_y, z_vec.data());

	 
	for (size_t i = 0; i < dim_x; i++) {
		x_arr[i] = x_vec[i];
	}
	for (size_t i = 0; i < dim_y; i++) {
		y_arr[i] = y_vec[i * dim_x];
	}

	 
	for (size_t i = 0; i < dim_x; i++) {
		for (size_t j = 0; j < dim_y; j++) {
			assert(x_vec[i + dim_x * j] == x_arr[i]);
			assert(y_vec[i + dim_x * j] == y_arr[j]);
		}
	}

	 
	alglib::spline2dinterpolant spline;
	try {
		alglib::spline2dbuildbilinearv(x_arr, dim_x, y_arr, dim_y, z_arr, 1, spline);
	} catch (alglib::ap_error e) {
		throw std::exception();
	}

	 
	 
	 
	 
	 
	return spline;
}

alglib::spline3dinterpolant get_spline_from_csv(const std::string& filename, const std::string& x_fieldname,
	const std::string& y_fieldname, const std::string& z_fieldname, const std::string& w_fieldname, size_t dim_x,
	size_t dim_y, size_t dim_z) {
	 
	std::ifstream file(filename);
	assert(file.good());

	 
	io::CSVReader<4> in(filename);
	in.read_header(io::ignore_extra_column, x_fieldname, y_fieldname, z_fieldname, w_fieldname);

	vector<double> x_vec;
	vector<double> y_vec;
	vector<double> z_vec;
	vector<double> w_vec;
	x_vec.reserve(dim_x * dim_y * dim_z);
	y_vec.reserve(dim_x * dim_y * dim_z);
	z_vec.reserve(dim_x * dim_y * dim_z);
	w_vec.reserve(dim_x * dim_y * dim_z);

	 
	assert(in.has_column(x_fieldname));
	assert(in.has_column(y_fieldname));
	assert(in.has_column(z_fieldname));
	assert(in.has_column(w_fieldname));

	 
	double x, y, z, w;
	while (in.read_row(x, y, z, w)) {
		x_vec.push_back(x);
		y_vec.push_back(y);
		z_vec.push_back(z);
		w_vec.push_back(w);

		 
		assert(!(isnan(x) || isnan(y) || isnan(z) || isnan(w)));
		assert(!(isinf(x) || isinf(y) || isinf(z) || isinf(w)));
	}

	assert(x_vec.size() == dim_x * dim_y * dim_z);

	 
	struct XYZW {
		double x;
		double y;
		double z;
		double w;
	};

	 
	std::vector<XYZW> xyzw_vec;
	xyzw_vec.reserve(x_vec.size());
	for (size_t i = 0; i < x_vec.size(); ++i) {
		xyzw_vec.push_back({x_vec[i], y_vec[i], z_vec[i], w_vec[i]});
	}

	auto comp = [](const auto& lhs, const auto& rhs) {
		if (lhs.z < rhs.z) {
			return true;
		} else if (lhs.z > rhs.z) {
			return false;
		} else if (lhs.y < rhs.y) {
			return true;
		} else if (lhs.y > rhs.y) {
			return false;
		}
		return lhs.x < rhs.x;
	};
	std::sort(xyzw_vec.begin(), xyzw_vec.end(), comp);
	for (size_t i = 0; i < xyzw_vec.size(); ++i) {
		x_vec[i] = xyzw_vec[i].x;
		y_vec[i] = xyzw_vec[i].y;
		z_vec[i] = xyzw_vec[i].z;
		w_vec[i] = xyzw_vec[i].w;
	}

	 
	alglib::real_1d_array x_arr;
	alglib::real_1d_array y_arr;
	alglib::real_1d_array z_arr;
	x_arr.setlength(dim_x);
	y_arr.setlength(dim_y);
	z_arr.setlength(dim_z);
	for (size_t i = 0; i < dim_x; ++i) {
		x_arr[i] = x_vec[i];
	}

	for (size_t i = 0; i < dim_y; ++i) {
		y_arr[i] = y_vec[i * dim_x];
	}

	for (size_t i = 0; i < dim_z; ++i) {
		z_arr[i] = z_vec[i * dim_x * dim_y];
	}

	 
	for (size_t i = 0; i < dim_x; ++i) {
		for (size_t j = 0; j < dim_y; ++j) {
			for (size_t k = 0; k < dim_z; ++k) {
				assert(x_vec[i + dim_x * j + dim_x * dim_y * k] == x_arr[i]);
				assert(y_vec[i + dim_x * j + dim_x * dim_y * k] == y_arr[j]);
				assert(z_vec[i + dim_x * j + dim_x * dim_y * k] == z_arr[k]);
			}
		}
	}

	alglib::real_1d_array w_arr;
	w_arr.setcontent(dim_x * dim_y * dim_z, &w_vec[0]);

	alglib::spline3dinterpolant spline;
	try {
		alglib::spline3dbuildtrilinearv(x_arr, dim_x, y_arr, dim_y, z_arr, dim_z, w_arr, 1, spline);
	} catch (alglib::ap_error e) {
		throw std::exception();
	}

	 
	 
	 
	 

	return spline;
}
