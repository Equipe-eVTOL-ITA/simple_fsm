cmake_minimum_required(VERSION 3.8)
project(drone_control)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


#find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(fsm REQUIRED)
find_package(cbr_drone_lib REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(include)


#takeoff_landing

add_executable(takeoff_landing
  src/takeoff_landing.cpp
)

ament_target_dependencies(takeoff_landing
  rclcpp
  fsm
  cbr_drone_lib
  Eigen3
)


#goto

add_executable(goto
  src/goto.cpp
)

ament_target_dependencies(goto
  rclcpp
  fsm
  cbr_drone_lib
  Eigen3
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS
    takeoff_landing
    goto
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
