cmake_minimum_required(VERSION 3.8)
project(pneutrunk_gui)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON) #This will autogenerate qt gui stuff
set(CMAKE_AUTORCC ON) #this needs to be on so cmake will know what to do with QT resource Files
set(QT_USE_QTMULTIMEDIA TRUE)
set(QT_USE_QTMULTIMEDIAWIDGETS TRUE)
# set(CMAKE_AUTOUIC ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2)  
find_package(rviz_common)  
find_package(rviz_default_plugins REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pneutrunk_msgs REQUIRED)
find_package(Eigen3 REQUIRED)



find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


set(QT_LIBRARIES Qt5::Widgets)


include_directories(include)
include_directories(${PROJECT_SOURCE_DIR}/Sources/)
include_directories(${PROJECT_SOURCE_DIR}/Headers/)
include_directories(${PROJECT_SOURCE_DIR}/Forms/)
# include_directories(${PROJECT_SOURCE_DIR}/fonts/)
# include_directories(${PROJECT_SOURCE_DIR}/icons/)


include_directories(${CMAKE_CURRENT_BINARY_DIR})
SET(SOURCES ${PROJECT_SOURCE_DIR}/Sources/main.cpp
        ${PROJECT_SOURCE_DIR}/Sources/mainwindow.cpp
        ${PROJECT_SOURCE_DIR}/Sources/ros_module.cpp)
SET(HEADERS ${PROJECT_SOURCE_DIR}/Headers/mainwindow.h ${PROJECT_SOURCE_DIR}/Headers/ros_module.h)
SET(FORMS ${PROJECT_SOURCE_DIR}/Forms/mainwindow.ui)
SET(RESOURCE_FILES ${PROJECT_SOURCE_DIR}/Icons/files.qrc)

QT5_WRAP_CPP(HEADERS_MOC ${HEADERS})
QT5_WRAP_UI(FORMS_HEADERS ${FORMS})
QT5_ADD_RESOURCES(RESOURCES ${RESOURCE_FILES})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



add_executable(${PROJECT_NAME} ${SOURCES} ${HEADERS} ${FORMS} ${HEADERS_MOC} ${FORMS_HEADERS} ${RESOURCES})
ament_target_dependencies(${PROJECT_NAME}
  "Qt5Core"
  "Qt5Widgets"
  "rclcpp"
  "rviz_common"
  "rviz_default_plugins"
  "rviz_rendering"
  pneutrunk_msgs
)

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} )

ament_export_dependencies(
  rclcpp
  rviz_common
  rviz_default_plugins
  rviz_rendering
  pneutrunk_msgs
  )

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
