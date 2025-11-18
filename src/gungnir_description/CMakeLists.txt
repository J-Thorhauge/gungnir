cmake_minimum_required(VERSION 3.8)
project(gungnir_description)

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)

install(
  DIRECTORY config meshes srdf urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
