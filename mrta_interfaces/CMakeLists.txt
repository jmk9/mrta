cmake_minimum_required(VERSION 3.8)
project(mrta_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Task.msg"
  "msg/RobotPlan.msg"
  "msg/RobotState.msg"
  "srv/ReserveCharge.srv"
  "srv/ReleaseCharge.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
