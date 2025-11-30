add_library(config_file "")
target_sources(config_file PRIVATE ConfigFile.cpp PUBLIC ConfigFile.h)

target_link_libraries(config_file PUBLIC tomlplusplus)
