# -----------------------------------------------------------------------------
# Lesson "5.3: Model Predictive Control"
# Authors     : David Siller, Andrew Gray, Dominique Luna.
#
# Modified by : Jonathan L. Moran (jonathan.moran107@gmail.com) 
#
# Purpose of this file: Define and configures the CMAKE build script.
# -----------------------------------------------------------------------------


message("========= Model Predictive Control (MPC) =========")
#message("========= Authored By Jonathan L. Moran  =========")
message("By Jonathan L. Moran (jonathan.moran107@gmail.com)")
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
# ```sh brew install ipopt```
# NOTE for macOS: install `cppad` with:
# ```sh brew install cppad```
# For issues on macOS, see:
# https://github.com/udacity/CarND-MPC-Project/issues/34
include_directories(./include/)
include_directories(./include/eigen-3.3.7)
include_directories(/usr/local/include)
include_directories(/usr/local/lib)
message("=========      Finding C++ packages      =========")
find_package(Eigen3 REQUIRED)
message("Eigen3_FOUND:${Eigen3_FOUND}")
message("Eigen3_VERSION:${Eigen3_VERSION}")
# find_package(CppAD REQUIRED)
# message("CppAD_FOUND:${CppAD_FOUND}")
# message("CppAD_VERSION:${CppAD_VERSION}")
# message("CppAD_INCLUDE_DIRS:${CppAD_INCLUDE_DIRS}")
message("=========    Finding Python packages     =========")
find_package(Python COMPONENTS NumPy REQUIRED Interpreter Development)
message("Python_FOUND:${Python_FOUND}")
message("Python_VERSION:${Python_VERSION}")
message("Python_INCLUDE_DIRS:${Python_INCLUDE_DIRS}")
message("Python_Development_FOUND:${Python_Development_FOUND}")
message("Python_LIBRARIES:${Python_LIBRARIES}")
message("NumPy_FOUND:${Python_NumPy_FOUND}")
message("Python_NumPy_VERSION:${Python_NumPy_VERSION}")
message("Python_NumPy_INCLUDE_DIRS:${Python_NumPy_INCLUDE_DIRS}")
message("========= Linking packages and libraries =========")
include_directories(${Python_INCLUDE_DIRS})
include_directories(${Python_NumPy_INCLUDE_DIRS})
#target_link_libraries(mpc ${PYTHON_LIBRARIES})
link_directories(/usr/local/lib)
### Adding header files and project-related files
#add_executable(./include/matplotlibcpp.h)
add_executable(mpc ${sources})
### Perform OS-specific linking 
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
target_link_libraries(mpc ipopt python2.7)
else()
#target_link_libraries(mpc ipopt python)
target_link_libraries(mpc Eigen3::Eigen ipopt cppad_lib ${Python_LIBRARIES})
endif()