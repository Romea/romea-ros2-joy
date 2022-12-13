cmake_minimum_required(VERSION 3.5)
project(romea_joystick_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config 
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_joystick_description.py)

ament_package()
