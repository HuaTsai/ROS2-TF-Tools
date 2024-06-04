cmake_minimum_required(VERSION 3.8)
project(tf2_gui_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
list(APPEND CMAKE_AUTOUIC_SEARCH_PATHS ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

string(TOUPPER $ENV{ROS_DISTRO} ROS_DISTRO)
add_definitions(-DROS_DISTRO_${ROS_DISTRO})

include_directories(include)

add_executable(main src/main.cc src/mainwindow.cc src/comm_node.cc res/resources.qrc)
target_link_libraries(main Qt5::Widgets)
ament_target_dependencies(main rclcpp tf2_eigen)

install(
  TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
