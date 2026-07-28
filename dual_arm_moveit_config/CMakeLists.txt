cmake_minimum_required(VERSION 3.8)
project(dual_arm_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config launch rviz hardware
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(
  PROGRAMS
    hardware/real_hardware.py
    hardware/teleop_bridge.py
    hardware/joint_state_filter.py
    hardware/isaac_joint_adapter.py
    hardware/isaac_state_relay.py
    dual_arm_moveit_config/exotica_ik_server_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_isaac_joint_mapping
    test/test_isaac_joint_mapping.py
  )
endif()

ament_package()
