cmake_minimum_required(VERSION 3.8)
project(pneutrunk_rviz_plugin)

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
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
find_package(rviz2)  
find_package(rviz_common)  
find_package(pluginlib REQUIRED)

include_directories(include ${rclcpp_INCLUDE_DIRS})

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED src/pneutrunk_plugin.cpp include/pneutrunk_rviz_plugin/pneutrunk_plugin.h)

# Link ament packages
ament_target_dependencies(${PROJECT_NAME} rclcpp rviz_common)

# Link non ament packages
target_link_libraries(${PROJECT_NAME}  Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME}  PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

install(TARGETS
 ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

ament_export_libraries(${PROJECT_NAME})

ament_package()
