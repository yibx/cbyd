cmake_minimum_required(VERSION 2.6)
project(cbyd)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=armv8-a -mtune=native -ffast-math -funroll-loops -fomit-frame-pointer")
add_definitions(-DEIGEN_NO_DEBUG -DEIGEN_VECTORIZE_NEON -DNDEBUG)
set(CMAKE_BUILD_TYPE "Debug")
find_package(PCL 1.13.0 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS}  ./include  ./third-party/libhv/include ./include/LSSDK/Include ./third-party/yaml/include)
link_directories(${PCL_LIBRARY_DIRS} ./lib ./third-party/yaml/lib)
add_definitions(${PCL_DEFINITIONS})
#add_library(test_pcl $(./LSSDK))

add_executable(cbyd ./src/main.cpp ./src/point_cloud_acquirer.cpp ./src/point_cloud_fuser.cpp
./src/six_dof_calculator.cpp ./src/data_outputter.cpp ./src/system_state_manager.cpp ./src/wharf_dock_checker.cpp ./src/Logger.cpp ./src/easylogging++.cc ./src/config_loader.cpp)
 
target_link_libraries (cbyd ${PCL_LIBRARIES} yaml-cpp pthread)
 
#install(TARGETS test_pcl RUNTIME DESTINATION bin)
