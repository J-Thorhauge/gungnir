cmake_minimum_required(VERSION 3.8)
project(gungnir_controller)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

include_directories(include)

add_library(gungnir_controller SHARED
  src/some_script.cpp
)


ament_target_dependencies(gungnir_controller
  rclcpp
  pluginlib
)

install(TARGETS gungnir_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
