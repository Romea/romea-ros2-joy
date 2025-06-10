cmake_minimum_required(VERSION 3.5)
project(romea_joystick_meta_bringup)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch profile
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_module(python/romea_joystick_meta_bringup.py)

install(PROGRAMS
  scripts/generate_launch_file.py
  scripts/generate_buttons_mapping_file.py
  scripts/generate_joystick_mapping_file.py
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  configure_file(test/test_joystick_meta_bringup.yaml test_joystick_meta_bringup.yaml)
  configure_file(test/test_buttons_mapping.yaml test_buttons_mapping.yaml)
  configure_file(test/test_joystick_mapping.yaml test_joystick_mapping.yaml)
  ament_add_pytest_test(test_joystick_meta_bringup test/test_joystick_meta_bringup.py)
  ament_add_pytest_test(test_buttons_mapping test/test_buttons_mapping.py)
  ament_add_pytest_test(test_joystick_mapping test/test_joystick_mapping.py)
endif()


ament_package()
