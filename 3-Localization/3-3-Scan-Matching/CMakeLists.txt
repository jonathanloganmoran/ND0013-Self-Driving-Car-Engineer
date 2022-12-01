project(Localization)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

set(sources icp1-main.cpp helper.cpp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(icp ${sources} )

target_link_libraries(icp ${PCL_LIBRARIES})
