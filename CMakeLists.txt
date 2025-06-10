cmake_minimum_required(VERSION 3.8)
project(fish_hpurv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

#caudalforceplugin
add_library(CaudalForcePlugin SHARED scripts/CaudalForcePlugin.cpp)

target_include_directories(CaudalForcePlugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(CaudalForcePlugin
  rclcpp
  std_msgs
  gazebo_ros
)

target_link_libraries(CaudalForcePlugin
  ${GAZEBO_LIBRARIES}
  ignition-math6::ignition-math6
)

install(TARGETS CaudalForcePlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)

add_library(FishBuoyancyPlugin SHARED scripts/FishBuoyancyPlugin.cpp)

target_include_directories(FishBuoyancyPlugin PUBLIC
  ${GAZEBO_INCLUDE_DIRS}
)

ament_target_dependencies(FishBuoyancyPlugin
  rclcpp
  std_msgs
  gazebo_ros
)

target_link_libraries(FishBuoyancyPlugin
  ${GAZEBO_LIBRARIES}
  ignition-math6::ignition-math6
)

install(TARGETS FishBuoyancyPlugin
  LIBRARY DESTINATION lib/${PROJECT_NAME}
)


# Install directories
install(DIRECTORY launch urdf config meshes worlds models
  DESTINATION share/${PROJECT_NAME}
)

# Install xacro files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.xacro"
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
