# -----------------------------------------------------------------------------
# Lesson "5.3: Model Predictive Control"
# Authors     : David Siller, Andrew Gray, Dominique Luna.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
#
# Purpose of this file: Define and configures the CMAKE build script.
# -----------------------------------------------------------------------------

project(mpc)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

### Setting the project sources
set(sources ./mpc.cc ./main.cc)

### Including relevant libraries:
# `eigen-3.3.71 and `matplotlibcpp.h`
# NOTE for macOS: install `ipopt` with:
# ```sh brew install ipopt --with-openblas```
include_directories(./include/)
include_directories(./include/eigen-3.3.7)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

### Adding header files and project-related files
add_executable(./include/matplotlibcpp.h)
add_executable(mpc ${sources})

### Perform OS-specific linking instructions 
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(mpc ipopt python2.7)
else()
target_link_libraries(mpc ipopt python)
endif()
