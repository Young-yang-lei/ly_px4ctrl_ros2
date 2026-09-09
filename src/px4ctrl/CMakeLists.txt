cmake_minimum_required(VERSION 3.8)
project(px4ctrl)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
  src/PX4CtrlFSM.cpp
)

target_include_directories(px4ctrl_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(px4ctrl_node
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
)

target_link_libraries(px4ctrl_node Eigen3::Eigen)

install(TARGETS px4ctrl_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  thrust_calibrate_scripts/thrust_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  thrust_calibrate_scripts/record.sh
  DESTINATION share/${PROJECT_NAME}/thrust_calibrate_scripts
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
