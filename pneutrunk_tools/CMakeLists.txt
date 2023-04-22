cmake_minimum_required(VERSION 3.8)
project(pneutrunk_tools)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pneutrunk_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)





include_directories(include)

add_executable(testovaci src/main.cpp src/continuum_model.cpp src/continuum_kinematics.cpp)
ament_target_dependencies(testovaci 
  rclcpp 
  std_msgs 
  visualization_msgs 
  pneutrunk_msgs 
  tf2
  tf2_ros
  geometry_msgs
  )

# rosidl_target_interfaces(testovaci ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS
  testovaci
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
