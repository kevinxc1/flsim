add_library(alglib "")

target_sources(
	alglib
	PRIVATE
		alglibinternal.cpp
		alglibmisc.cpp
		ap.cpp
		dataanalysis.cpp
		diffequations.cpp
		fasttransforms.cpp
		integration.cpp
		interpolation.cpp
		kernels_avx2.cpp
		kernels_fma.cpp
		kernels_sse2.cpp
		linalg.cpp
		optimization.cpp
		solvers.cpp
		specialfunctions.cpp
		statistics.cpp
	PUBLIC
		alglibinternal.h
		alglibmisc.h
		ap.h
		dataanalysis.h
		diffequations.h
		fasttransforms.h
		integration.h
		interpolation.h
		kernels_avx2.h
		kernels_fma.h
		kernels_sse2.h
		linalg.h
		optimization.h
		solvers.h
		specialfunctions.h
		statistics.h
		stdafx.h
)

if(WIN32 OR MSVC)
	add_compile_definitions(AE_OS=AE_WINDOWS)
elseif(UNIX)
	add_compile_definitions(AE_OS=AE_POSIX)
	# When compiling on POSIX/Linux, link ALGLIB with libpthread library. This is needed for
	# multithreaded optimization.
	target_link_libraries(alglib PRIVATE pthread)
endif()

# check architecture and print it out: x86_64, i386, arm, arm64, etc.
message(STATUS "CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")

target_include_directories(alglib INTERFACE ${PROJECT_SOURCE_DIR}/external)

# Compile with no warnings and with -O3 (regardless of build type)
target_compile_options(
	alglib
	PRIVATE
		-w
		-O3
)