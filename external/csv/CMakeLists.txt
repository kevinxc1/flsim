add_library(csv INTERFACE csv.h)

target_include_directories(csv INTERFACE ${PROJECT_SOURCE_DIR}/external)
