#include "ConfigFile.h"

#include <cstddef>
#include <exception>
#include <optional>
#include <ostream>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "Tools/FileTools.h"
#include "external/toml/toml.hpp"

namespace {
	ConfigFile::ConfigType get_file_type(std::string_view file_path) {
		constexpr std::string_view toml_file_extension = "toml";
		const std::string file_extension = file_tools::get_file_extension(file_path);
		if (file_extension == toml_file_extension) {
			return ConfigFile::ConfigType::TOML;
		}
		throw std::exception();
	}
}   

ConfigFile::ConfigFile() : config_type(default_config_type) {}

ConfigFile::ConfigFile(toml::table config) : config_type(ConfigType::TOML), toml_config(std::move(config)) {}

std::optional<ConfigFile> ConfigFile::from_toml(std::string_view toml_string) noexcept {
	const toml::parse_result result = toml::parse(toml_string);
	if (!result) {
		return std::nullopt;
	}
	return std::move(result).table();
}

std::optional<ConfigFile> ConfigFile::from_path(std::string_view file_path) {
	const ConfigType config_type = get_file_type(file_path);
	switch (config_type) {
		case ConfigType::TOML: {
			const toml::parse_result result = toml::parse_file(file_path);
			if (!result) {
				return std::nullopt;
			}
			return std::move(result).table();
		}
		default:
			return std::nullopt;
	}
	return std::nullopt;
}

bool ConfigFile::contains(std::string_view key) const noexcept {
	switch (config_type) {
		case ConfigType::TOML:
			return toml_config.contains(key.data());
		default:
			return false;
	}
	return false;
}

template <>
std::optional<ConfigFile> ConfigFile::get<ConfigFile>(std::string_view key) const noexcept {
	switch (config_type) {
		case ConfigType::TOML: {
			const toml::table* table = toml_config.at(key.data()).as_table();
			if (table == nullptr) {
				return std::nullopt;
			}
			return ConfigFile(*table);
		}
		default:
			return std::nullopt;
	}
	return std::nullopt;
}

template <>
std::optional<std::vector<ConfigFile>> ConfigFile::get_array<ConfigFile>(std::string_view key) const noexcept {
	switch (config_type) {
		case ConfigType::TOML: {
			const toml::array* array = toml_config.at_path(key).as_array();
			if (array == nullptr) {
				return std::nullopt;
			}
			std::vector<ConfigFile> configs;
			for (const auto& element : *array) {
				const toml::table* table = element.as_table();
				if (table == nullptr) {
					continue;
				}
				configs.emplace_back(*table);
			}
			return configs;
		}
		default:
			return std::nullopt;
	}
	return std::nullopt;
}

template <>
std::optional<ConfigFile> ConfigFile::get_path<ConfigFile>(std::string_view full_key) const noexcept {
	switch (config_type) {
		case ConfigType::TOML: {
			const toml::table* table = toml_config.at_path(full_key).as_table();
			if (table == nullptr) {
				return std::nullopt;
			}
			return ConfigFile(*table);
		}
		default:
			return std::nullopt;
	}
	return std::nullopt;
}

template <>
std::optional<std::vector<ConfigFile>> ConfigFile::get_path_array<ConfigFile>(
	std::string_view full_key) const noexcept {
	switch (config_type) {
		case ConfigType::TOML: {
			const toml::array* array = toml_config.at_path(full_key).as_array();
			if (array == nullptr) {
				return std::nullopt;
			}
			std::vector<ConfigFile> configs;
			configs.reserve(array->size());
			for (const auto& element : *array) {
				const toml::table* table = element.as_table();
				if (table == nullptr) {
					continue;
				}
				configs.emplace_back(*table);
			}
			return configs;
		}
		default:
			return std::nullopt;
	}
	return std::nullopt;
}

template <>
void ConfigFile::set<ConfigFile>(std::string_view key, ConfigFile value) {
	switch (config_type) {
		case ConfigType::TOML: {
			switch (value.config_type) {
				case ConfigType::TOML: {
					toml_config.insert_or_assign(key, value.toml_config);
					break;
				}
				default:
					throw std::exception();
			}
			break;
		}
		default:
			throw std::exception();
	}
}

template <>
void ConfigFile::set_array<ConfigFile>(std::string_view key, std::span<const ConfigFile> values) {
	switch (config_type) {
		case ConfigType::TOML: {
			toml::array array;
			for (const ConfigFile& value : values) {
				switch (value.config_type) {
					case ConfigType::TOML: {
						array.push_back(value.toml_config);
						break;
					}
					default:
						throw std::exception();
				}
			}
			toml_config.insert_or_assign(key, array);
			break;
		}
		default:
			throw std::exception();
	}
}

template <>
void ConfigFile::insert_override_l<ConfigFile>(std::string_view full_key, const ConfigFile& value) {
	std::vector<std::string_view> keys = split_key_by(full_key, '.');
	switch (config_type) {
		case ConfigType::TOML: {
			toml::table* table = traverse_create_toml(keys);
			table->insert_or_assign(keys.back().data(), value.toml_config);
			break;
		}
		default:
			throw std::exception();
	}
}

void ConfigFile::insert_override(const ConfigFile& config_file) {
	switch (config_file.config_type) {
		case ConfigType::TOML: {
			merge(toml_config, config_file.toml_config);
			break;
		}
		default:
			throw std::exception();
	}
}

template <>
void ConfigFile::insert_override_arr_l<ConfigFile>(std::string_view full_key, std::span<const ConfigFile> values) {
	std::vector<std::string_view> keys = split_key_by(full_key, '.');
	switch (config_type) {
		case ConfigType::TOML: {
			toml::table* table = traverse_create_toml(keys);
			toml::array array;
			for (const auto& value : values) {
				array.push_back(value.toml_config);
			}
			table->insert_or_assign(keys.back().data(), array);
			break;
		}
		default:
			throw std::exception();
	}
}

std::vector<std::string_view> ConfigFile::split_key_by(std::string_view key, char delimiter) noexcept {
	std::vector<std::string_view> keys;
	while (key.find(delimiter) != std::string_view::npos) {
		keys.push_back(key.substr(0, key.find(delimiter)));
		key = key.substr(key.find(delimiter) + 1);
	}
	keys.push_back(key);
	return keys;
}

std::string_view ConfigFile::get_file_extension() const noexcept {
	constexpr std::string_view toml_file_extension = "toml";
	constexpr std::string_view error_extension = "";   
	switch (config_type) {
		case ConfigType::TOML:
			return toml_file_extension;
		default:
			return error_extension;
	}
	return error_extension;
}

std::string ConfigFile::to_string() const noexcept {
	switch (config_type) {
		case ConfigType::TOML: {
			std::stringstream stream;
			stream << toml_config;
			return stream.str();
		}
		default:
			return "";
	}
	return "";
}

std::ostream& operator<<(std::ostream& ostream, const ConfigFile& config_file) {
	switch (config_file.config_type) {
		case ConfigFile::ConfigType::TOML:
			return ostream << config_file.toml_config;
		default:
			throw std::exception();
	}
	return ostream;
}

toml::table* ConfigFile::traverse_create_toml(std::span<std::string_view> keys) {
	toml::table* table = &toml_config;
	for (size_t i = 0; i < keys.size() - 1; i++) {
		const auto& key = keys[i];
		if (table->contains(key)) {
			table = table->at(key).as_table();
			if (table != nullptr) {
				continue;
			}
		}
		table = table->insert_or_assign(key, toml::table()).first->second.as_table();
	}
	return table;
}

void ConfigFile::merge(toml::table& table, const toml::table& other_table) {
	for (const auto& [key, value] : other_table) {
		if (table.contains(key)) {
			if (table.at(key).is_table() && value.is_table()) {
				merge(*table.at(key).as_table(), *value.as_table());
				continue;
			}
			table.insert_or_assign(key, value);
			continue;
		}
		table.insert_or_assign(key, value);
	}
}
