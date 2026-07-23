cmake_minimum_required(VERSION 3.8)
project(vacuum_robot_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch maps models rviz scripts urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    robot_description_contract
    test/test_robot_description.py
    TIMEOUT 30
  )
endif()

ament_package()
