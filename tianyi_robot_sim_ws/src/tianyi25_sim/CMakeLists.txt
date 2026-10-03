cmake_minimum_required(VERSION 3.8)
project(tianyi25_sim)

find_package(ament_cmake REQUIRED)

# 仿真资源整体安装到 share/${PROJECT_NAME}/ 下，供 launch 中的 get_package_share_directory 引用
install(DIRECTORY worlds launch config models urdf
  DESTINATION share/${PROJECT_NAME}
)

# 演示/工具节点安装到 lib/${PROJECT_NAME}/，供 ros2 run 与 launch 的 Node(executable=...) 调用
install(PROGRAMS scripts/joint_sweep_demo.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_sweep_demo
)

# Gazebo 侧关节扫掠：向内置 JointTrajectoryController 下发轨迹
install(PROGRAMS scripts/gz_joint_sweep.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_joint_sweep
)

# 里程计 TF 桥：把 gz 的 /odom 转成 odom -> base_footprint
install(PROGRAMS scripts/gz_odom_tf.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME gz_odom_tf
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()