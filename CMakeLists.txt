cmake_minimum_required(VERSION 2.8.3)
project(pomdp_ros)
add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg tf nav_msgs)

find_package(asif++ CONFIG REQUIRED)

find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

add_definitions(${COMPILE_FLAGS})

include_directories(include ${catkin_INCLUDE_DIRS}  /usr/include/eigen3)
link_directories(boost_system)

add_executable(integrator src/integrator.cpp)
target_link_libraries(integrator ${catkin_LIBRARIES} boost_system)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})

add_executable(grid_states src/grid_states.cpp)
target_link_libraries(grid_states ${catkin_LIBRARIES})

add_executable(gazebo_interface src/gazebo_interface.cpp)
target_link_libraries(gazebo_interface ${catkin_LIBRARIES})

add_executable(action_commander src/action_commander.cpp)
target_link_libraries(action_commander ${catkin_LIBRARIES})
