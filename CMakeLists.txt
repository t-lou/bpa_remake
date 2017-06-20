cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
project(bpa_remake)
#set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} include)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
add_executable(bpa_remake
    src/main.cpp
    src/Edge.cpp
    src/Front.cpp
    src/Pivoter.cpp
    src/MlsApplier.cpp)
target_link_libraries(bpa_remake ${PCL_LIBRARIES})
