cmake_minimum_required(VERSION 3.14)

project(MachEngine)

# enable C++17 features
if(CMAKE_COMPILER_IS_GNUCXX)
	message(STATUS "Using GNU C++17")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++20")
endif()
if(MSVC)
	message(STATUS "Using MSCV C++17")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++20")
endif(MSVC)

add_subdirectory(engine)
add_subdirectory(sandbox)