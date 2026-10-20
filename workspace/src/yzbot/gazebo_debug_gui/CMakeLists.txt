cmake_minimum_required(VERSION 3.8)
project(gazebo_debug_gui)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)

add_library(gazebo_debug_gui_plugin SHARED
  src/gazebo_debug_gui_plugin.cpp
)
target_compile_features(gazebo_debug_gui_plugin PUBLIC cxx_std_17)
target_compile_options(gazebo_debug_gui_plugin PRIVATE
  -Wall
  -Wextra
  -Wpedantic
)
target_include_directories(gazebo_debug_gui_plugin PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_directories(gazebo_debug_gui_plugin PRIVATE
  ${GAZEBO_LIBRARY_DIRS}
)
target_link_libraries(gazebo_debug_gui_plugin
  ${GAZEBO_LIBRARIES}
  Qt5::Core
  Qt5::Widgets
)

install(TARGETS gazebo_debug_gui_plugin
  LIBRARY DESTINATION lib
)

ament_package()
