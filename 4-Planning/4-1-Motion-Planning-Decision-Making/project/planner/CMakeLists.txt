project(MotionPlanning)

cmake_minimum_required (VERSION 3.10)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++14 -O3)

set(CXX_FLAGS "-pthread -fPIC -DNDEBUG -Wall -Wextra")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS} -Wl,-Bstatic -lcarla_client -lrpc -lboost_filesystem -Wl,-Bdynamic")

set(sources main.cpp behavior_planner_FSM.cpp motion_planner.cpp cubic_spiral.cpp spiral_base.cpp integral.cpp spiral_equations.cpp cost_functions.cpp utils.cpp velocity_profile_generator.cpp )

set(CARLALIB_DIR ${CMAKE_SOURCE_DIR}/libcarla-install/lib)
set(carla_libs  ${CARLALIB_DIR}/libboost_filesystem.a ${CARLALIB_DIR}/libboost_numpy36.a ${CARLALIB_DIR}/libboost_program_options.a ${CARLALIB_DIR}/libboost_python36.a ${CARLALIB_DIR}/libboost_system.a ${CARLALIB_DIR}/libcarla_client.a ${CARLALIB_DIR}/librpc.a ${CARLALIB_DIR}/libDebugUtils.a ${CARLALIB_DIR}/libDetour.a ${CARLALIB_DIR}/libDetourCrowd.a ${CARLALIB_DIR}/libDetourTileCache.a ${CARLALIB_DIR}/libRecast.a)
set(gtest_lib /usr/src/gtest/libgtest.a)

include_directories(/usr/local/include)
link_directories(/usr/local/lib)

include_directories(eigen-3.3.7)

include_directories(libcarla-install/include)
include_directories(rpclib/include)
include_directories(libcarla-install/include/system)

link_directories(libcarla-install/lib)
include_directories(libcarla-install/include)

include_directories(../../Build/boost-1.72.0-c8-install/include)
link_directories(../../Build/boost-1.72.0-c8-install/lib)

include_directories(../../Build/rpclib-v2.2.1_c3-c8-libcxx-install/include)
link_directories(../../Build/rpclib-v2.2.1_c3-c8-libcxx-install/lib)

include_directories(../../Build/recast-cdce4e-c8-install/include)
link_directories(../../Build/recast-cdce4e-c8-install/lib)

include_directories(../../Build/)
link_directories(../../Build/)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(spiral_planner ${sources} )

target_link_libraries(spiral_planner z ssl uv uWS ${carla_libs} ${gtest_lib} glog boost_filesystem lapack)
