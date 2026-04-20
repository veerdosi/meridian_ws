cmake_minimum_required(VERSION 3.8)
project(meridian_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY assets/
  DESTINATION share/${PROJECT_NAME}/assets
)

ament_package()
