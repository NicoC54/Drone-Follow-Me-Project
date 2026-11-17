cmake_minimum_required(VERSION 3.16)
project(follow_me_gazebo LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(follow_me_core REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)

# Flight controller "firmware" of the follower: a Gazebo system plugin running the
# follow_me_core navigation (ESKF) + attitude/rate control + motor model in the physics loop.
add_library(follow_me_flight_controller SHARED src/flight_controller.cpp)
target_link_libraries(follow_me_flight_controller
  follow_me_core::follow_me_core
  gz-sim8::gz-sim8
  gz-plugin2::register
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10)
install(TARGETS follow_me_flight_controller LIBRARY DESTINATION lib)

# Sourcing install/setup.bash puts lib/ on GZ_SIM_SYSTEM_PLUGIN_PATH.
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/follow_me_gazebo.dsv.in")

install(DIRECTORY worlds config launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
