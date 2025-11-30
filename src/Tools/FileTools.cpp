#include "FileTools.h"

#include <algorithm>
#include <filesystem>
#include <string>

#include "Tools/RootDirectory.h"

namespace {
	std::string to_lower(std::string_view non_lower_string) {
		std::string return_value;
		return_value.resize(non_lower_string.size());
		std::transform(non_lower_string.begin(), non_lower_string.end(), return_value.begin(), ::tolower);
		return return_value;
	}
}   

bool file_tools::has_extension(std::string_view file_name, std::string_view extension) {
	const std::string current_extension = get_file_extension(file_name);
	const std::string expected_extension = to_lower(extension);
	return current_extension == extension;
}

std::string file_tools::get_file_extension(std::string_view file_name) {
	const std::string_view extension_substr = file_name.substr(file_name.find_last_of('.') + 1);
	std::string return_value;
	return_value.resize(extension_substr.size());
	std::transform(extension_substr.begin(), extension_substr.end(), return_value.begin(), ::tolower);
	return return_value;
}

std::vector<std::string> file_tools::get_files_in_directory(std::string_view directory, std::string_view extension) {
	std::vector<std::string> files;
	for (const auto& entry : std::filesystem::directory_iterator(directory)) {
		const std::filesystem::path path = normalize_path(entry.path().string());
		if (!std::filesystem::is_regular_file(path) || !has_extension(path.string(), extension)) {
			continue;
		}
		files.push_back(path.string());
	}
	return files;
}