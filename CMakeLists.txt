cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(bpa_remake)

set(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS}
  include)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(bpa_remake
  src/main.cpp)

target_link_libraries(bpa_remake
  ${PCL_LIBRARIES} stdc++fs)
