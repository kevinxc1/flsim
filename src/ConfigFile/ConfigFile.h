#ifndef MINISIM_OPTIMIZERCONFIG_H
#define MINISIM_OPTIMIZERCONFIG_H

#include <cassert>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#define TOML_EXCEPTIONS 0
#include "external/toml/toml.hpp"

class ConfigFile {
   public:
	enum class ConfigType : std::uint8_t { TOML };

	/// @brief Constructs an empty ConfigFile, using toml
	ConfigFile();
	ConfigFile(toml::table config);

	static std::optional<ConfigFile> from_path(std::string_view file_path);
	static std::optional<ConfigFile> from_toml(std::string_view toml_string) noexcept;

	/// @brief gets the value at the given key, if it exists and is of the given type
	template <typename T>
	std::optional<T> get(std::string_view key) const noexcept {
		switch (config_type) {
			case ConfigType::TOML:
				return toml_config.at_path(key).value<T>();
			default:
				return std::nullopt;
		}
		assert(false);
		return std::nullopt;  // For compiler when NDEBUG present
	}

	/// @brief force gets the value at the given key: fatal if it does not exist or is not of the given type
	template <typename T>
	T get_force(std::string_view key) const {
		std::optional<T> value = get<T>(key);
		if (!value) {
			throw std::exception();
		}
		return value.value();
	}

	/// @brief gets the value of an array at the given key, if it exists and is of the given type
	template <typename T>
	std::optional<std::vector<T>> get_array(std::string_view key) const noexcept {
		switch (config_type) {
			case ConfigType::TOML: {
				const toml::array* array = toml_config.at_path(key).as_array();
				return array ? std::make_optional(toml_array_to_vector<T>(array)) : std::nullopt;
			}
			default:
				throw std::exception();
		}
		assert(false);
		return std::nullopt;  // For compiler when NDEBUG present
	}

	/// @brief force gets the value of an array at the given key, fatal if it does not exist or is not of the given type
	template <typename T>
	std::vector<T> get_array_force(std::string_view key) const {
		std::optional<std::vector<T>> value = get_array<T>(key);
		if (!value) {
			throw std::exception();
		}
		return value.value();
	}

	/// @brief gets the value at the given key, splitting on the given delimiter (default: '.')
	template <typename T>
	std::optional<T> get_path(std::string_view full_key) const noexcept {
		switch (config_type) {
			case ConfigType::TOML:
				return toml_config.at_path(full_key).value<T>();
			default:
				throw std::exception();
		}
		assert(false);
		return std::nullopt;  // For compiler when NDEBUG present
	}

	/// @brief gets the value at the given key, recursively going through an args... list of keys (strings)
	template <typename T, typename... Args>
	std::optional<T> get_var(Args... keys) const noexcept {
		// use get_path with delimiter '.' to get the value
		std::ostringstream full_key;
		static_cast<void>(std::initializer_list<int>{(full_key << keys << '.', 0)...});
		std::string sol = full_key.str();
		sol.pop_back();
		return get_path<T>(sol);
	}

	/// @brief force gets the value at the given key, splitting on the given delimiter (default: '.')
	template <typename T>
	T get_path_force(std::string_view full_key) const {
		std::optional<T> value = get_path<T>(full_key);
		if (!value) {
			throw std::exception();
		}
		return value.value();
	}

	/// @brief force gets the value at the given key, splitting on the given delimiter (default: '.')
	template <typename T, typename... Args>
	T get_var_force(Args... keys) const {
		std::optional<T> value = get_var<T>(keys...);
		if (!value) {
			std::ostringstream full_key;
			static_cast<void>(std::initializer_list<int>{(full_key << keys << '.', 0)...});
			std::string sol = full_key.str();
			sol.pop_back();
			throw std::exception();
		}
		return value.value();
	}

	/// @brief gets the array at the given key recursively going through an args... list of keys (strings)
	template <typename T>
	std::optional<std::vector<T>> get_path_array(std::string_view full_key) const noexcept {
		switch (config_type) {
			case ConfigType::TOML: {
				const toml::array* array = toml_config.at_path(full_key).as_array();
				return array ? std::make_optional(toml_array_to_vector<T>(array)) : std::nullopt;
			}
			default:
				throw std::exception();
		}
		return std::nullopt;
	}

	/// @brief gets the array at the given key, splitting on the given delimiter (default: '.')
	template <typename T>
	std::vector<T> get_path_array_force(std::string_view full_key) const {
		std::optional<std::vector<T>> value = get_path_array<T>(full_key);
		if (!value) {
			throw std::exception();
		}
		return value.value();
	}

	/// @brief gets whether the given key exists
	bool contains(std::string_view key) const noexcept;

	/// @brief sets the value at the given key
	template <typename T>
	void set(std::string_view key, T value) {
		switch (config_type) {
			case ConfigType::TOML:
				toml_config.insert_or_assign(key.data(), value);
				break;
			default:
				throw std::exception();
		}
	}

	/// @brief sets an array at the given key
	template <typename T>
	void set_array(std::string_view key, std::span<const T> values) {
		switch (config_type) {
			case ConfigType::TOML: {
				toml::array array;
				for (const auto& value : values) {
					array.push_back(value);
				}
				toml_config.insert_or_assign(key.data(), array);
				break;
			}
			default:
				throw std::exception();
		}
	}

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override(std::string_view full_key, const T& value) {
		insert_override_l(full_key, value);
	}

	/// @brief r-value inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override(std::string_view full_key, const T&& value) {
		insert_override_l(full_key, value);
	}

	/// @brief inserts the given ConfigFile into this ConfigFile
	void insert_override(const ConfigFile& config_file);

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override_arr(std::string_view full_key, const std::span<const T> value) {
		insert_override_arr_l<T>(full_key, value);
	}

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T, typename... T2>
	void insert_override_array(std::string_view full_key, const T& value, const T2&... values) {
		std::vector<T> values_vec = {value};
		(values_vec.push_back(values), ...);
		insert_override_arr_l<T>(full_key, values_vec);
	}

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override_array(std::string_view full_key, const std::span<const T>&& value) {
		insert_override_arr_l(full_key, value);
	}

	/// @brief gets the expected file extension for the config type
	std::string_view get_file_extension() const noexcept;

	/// @brief prints the config file to a string
	std::string to_string() const noexcept;

	std::optional<toml::table> get_toml() const noexcept {
		if (config_type == ConfigType::TOML) {
			return toml_config;
		}
		return std::nullopt;
	}

	const toml::table& get_toml_force() const {
		if (config_type != ConfigType::TOML) {
			throw std::exception();
		}
		return toml_config;
	}

	friend std::ostream& operator<<(std::ostream& os, const ConfigFile& config_file);

   private:
	ConfigType config_type;
	static constexpr ConfigType default_config_type = ConfigType::TOML;

	toml::table toml_config;

	template <typename T>
	static std::vector<T> toml_array_to_vector(const toml::array* array) {
		assert(array != nullptr);
		if (!array->is_homogeneous()) {
			throw std::exception();
		}
		std::vector<T> vector;
		for (const auto& element : *array) {
			std::optional<T> value = element.value<T>();
			if (value.has_value()) {
				vector.push_back(value.value());
			}
		}
		return vector;
	}

	template <typename T>
	static toml::array span_to_toml_arr(std::span<const T> values) noexcept {
		toml::array array;
		for (const auto& value : values) {
			array.push_back(value);
		}
		return array;
	}

	static std::vector<std::string_view> split_key_by(std::string_view key, char delimiter) noexcept;

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override_l(std::string_view full_key, const T& value) {
		std::vector<std::string_view> keys = split_key_by(full_key, '.');
		switch (config_type) {
			case ConfigType::TOML: {
				toml::table* table = traverse_create_toml(keys);
				table->insert_or_assign(keys.back().data(), value);
				break;
			}
			default:
				throw std::exception();
		}
	}

	/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
	///
	/// Note: This overrides any existing values at the given keys if necessary
	template <typename T>
	void insert_override_arr_l(std::string_view full_key, std::span<const T> values) {
		std::vector<std::string_view> keys = split_key_by(full_key, '.');
		switch (config_type) {
			case ConfigType::TOML: {
				toml::table* table = traverse_create_toml(keys);
				table->insert_or_assign(keys.back().data(), span_to_toml_arr(values));
				break;
			}
			default:
				throw std::exception();
		}
	}

	/// @brief traverses the config toml file to the given key (creating intermediary keys if necessary), and returns a
	/// pointer to the value above the final key.
	///
	/// NOTE: this is a destructive operation, and will change the config file. You could lose data
	toml::table* traverse_create_toml(std::span<std::string_view> keys);

	/// @brief merges the given table into the other table, deep merging any nested tables
	static void merge(toml::table& table, const toml::table& other_table);
};

/// @brief gets the value at the given key, if it exists and is of the given type
template <>
std::optional<ConfigFile> ConfigFile::get<ConfigFile>(std::string_view key) const noexcept;

/// @brief gets the value of an array at the given key, if it exists and is of the given type
template <>
std::optional<std::vector<ConfigFile>> ConfigFile::get_array<ConfigFile>(std::string_view key) const noexcept;

/// @brief force gets the array at the given key, splitting on the given delimiter (default: '.')
template <>
std::optional<std::vector<ConfigFile>> ConfigFile::get_path_array(std::string_view full_key) const noexcept;

/// @brief sets the value at the given key
template <>
void ConfigFile::set<ConfigFile>(std::string_view key, ConfigFile values);

/// @brief sets an array at the given key
template <>
void ConfigFile::set_array<ConfigFile>(std::string_view key, std::span<const ConfigFile> values);

/// @brief gets the value at the given key, splitting on the given delimiter (default: '.')
template <>
std::optional<ConfigFile> ConfigFile::get_path<ConfigFile>(std::string_view full_key) const noexcept;

template <>
void ConfigFile::insert_override_l<ConfigFile>(std::string_view full_key, const ConfigFile& value);

/// @brief inserts the given initializer list of key-value pairs, nesting after every "." in the key
///
/// Note: This overrides any existing values at the given keys if necessary
template <>
void ConfigFile::insert_override_arr_l<ConfigFile>(std::string_view full_key, std::span<const ConfigFile> values);

#endif  // MINISIM_OPTIMIZERCONFIG_H
