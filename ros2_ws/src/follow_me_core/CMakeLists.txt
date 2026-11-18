cmake_minimum_required(VERSION 3.16)
project(follow_me_core LANGUAGES CXX)

# Pure C++ library (Eigen only, no ROS). ament_cmake is used only as the build system,
# so that the ROS packages can find it with find_package(follow_me_core).

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}
  src/camera_model.cpp
  src/ekf.cpp
  src/track_manager.cpp
  src/timed_tracker.cpp
  src/guidance.cpp
  src/controller.cpp
  src/sim_models.cpp
  src/quadrotor.cpp
  src/attitude_control.cpp
  src/nav_sensors.cpp
  src/nav_eskf.cpp
  src/radar_model.cpp
  src/fusion_tracker.cpp
  src/mission.cpp
  src/intercept_sim.cpp
  src/offline_sim.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
# Static library linked into the Gazebo plugin (a shared object): needs -fPIC.
set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(follow_me_offline_sim src/offline_sim_main.cpp)
target_link_libraries(follow_me_offline_sim ${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
install(TARGETS follow_me_offline_sim DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core ${PROJECT_NAME})
  ament_add_gtest(test_quadrotor test/test_quadrotor.cpp)
  target_link_libraries(test_quadrotor ${PROJECT_NAME})
  ament_add_gtest(test_nav test/test_nav.cpp)
  target_link_libraries(test_nav ${PROJECT_NAME})
  ament_add_gtest(test_radar test/test_radar.cpp)
  target_link_libraries(test_radar ${PROJECT_NAME})
  ament_add_gtest(test_fusion test/test_fusion.cpp)
  target_link_libraries(test_fusion ${PROJECT_NAME})
  ament_add_gtest(test_mission test/test_mission.cpp)
  target_link_libraries(test_mission ${PROJECT_NAME})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3)
ament_package()
