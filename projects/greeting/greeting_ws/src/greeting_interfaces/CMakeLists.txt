cmake_minimum_required(VERSION 3.8)
project(greeting_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ConsoleCommand.msg"
  "msg/GreetingEvent.msg"
  "msg/PersonDetection.msg"
  "msg/GreetingIntent.msg"
  "msg/SubsystemHealth.msg"
  "srv/GreetingControl.srv"
  "action/Speak.action"
  "action/NavigateTo.action"
  "action/PlayMotion.action"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()