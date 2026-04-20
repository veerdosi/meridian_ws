cmake_minimum_required(VERSION 3.8)
project(meridian_control)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionOutcome.msg"
  "msg/ComplianceState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install compliance_controller.py into the same site-packages dir rosidl uses.
# We cannot call ament_python_install_package() here — rosidl already creates
# that CMake target internally. Instead we query the install dir and use
# install(FILES) directly.
ament_get_python_install_dir(_py_install_dir)
install(FILES
  meridian_control/compliance_controller.py
  DESTINATION "${_py_install_dir}/${PROJECT_NAME}"
)

install(PROGRAMS
  node_scripts/compliance_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
