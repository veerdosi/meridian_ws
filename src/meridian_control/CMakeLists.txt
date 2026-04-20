cmake_minimum_required(VERSION 3.8)
project(meridian_control)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionOutcome.msg"
  "msg/ComplianceState.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install compliance_controller.py into the Python package that rosidl creates.
# ament_python_install_package() conflicts with rosidl's internal Python targets,
# so we install directly to the dist-packages path.
set(_py_install_dir
  "local/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/dist-packages")
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
