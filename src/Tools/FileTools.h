#ifndef MINISIM_FILETOOLS_H
#define MINISIM_FILETOOLS_H

#include <string_view>
#include <vector>

namespace file_tools {
	std::string get_file_extension(std::string_view file_name);
	bool has_extension(std::string_view file_name, std::string_view extension);
	/// @brief Gets all the files in a directory with a certain extension
	std::vector<std::string> get_files_in_directory(std::string_view directory, std::string_view extension);

}  // namespace file_tools

#endif  // MINISIM_FILETOOLS_H
