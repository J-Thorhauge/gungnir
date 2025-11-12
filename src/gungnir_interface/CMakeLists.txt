cmake_minimum_required(VERSION 3.8)
project(gungnir_interface)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(gungnir_interface SHARED
  src/some_script.cpp
)


ament_target_dependencies(gungnir_interface
  rclcpp
  pluginlib
)

install(TARGETS gungnir_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
