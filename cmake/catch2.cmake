CPMAddPackage("https://github.com/catchorg/Catch2.git@3.6.0")

FetchContent_MakeAvailable(Catch2)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)

