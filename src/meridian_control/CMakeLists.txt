cmake_minimum_required(VERSION 3.8)
project(meridian_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionOutcome.msg"
  "msg/ComplianceState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
