cmake_minimum_required(VERSION 3.8)
project(tianyi25_urdf)

find_package(ament_cmake REQUIRED)

# 资源目录整体安装到 share/${PROJECT_NAME}/ 下，供 URDF 中的 package:// 引用
install(DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()