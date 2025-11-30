#include "Tools/RootDirectory.h"

#include <mutex>

void RootDirectory::set_root_directory(std::string_view new_root_directory) {
	const std::lock_guard<std::mutex> lock(this->root_directory_mutex);
	this->root_directory = new_root_directory;
	this->root_directory = this->root_directory.lexically_normal();
}

std::string RootDirectory::get_root_directory() const {
	return this->root_directory.string();
}

std::string RootDirectory::prepend_root_directory(std::string_view path) const {
	return (this->root_directory / path).lexically_normal().string();
}

std::string RootDirectory::remove_root_directory_from(std::string_view path) const {
	const std::filesystem::path to_return = path;
	return to_return.lexically_relative(this->root_directory).string();
}

std::string get_root_directory() {
	return RootDirectory::get_instance()->get_root_directory();
}

std::string normalize_path(std::string_view path) {
	return RootDirectory::get_instance()->prepend_root_directory(path);
}

std::string remove_root_directory_from(std::string_view path) {
	return RootDirectory::get_instance()->remove_root_directory_from(path);
}

void set_root_directory(std::string_view new_root_directory) {
	RootDirectory::get_instance()->set_root_directory(new_root_directory);
}
