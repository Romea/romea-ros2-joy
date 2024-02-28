cmake_minimum_required(VERSION 3.5)
project(romea_joystick_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(romea_core_common REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
      src/joystick.cpp
      src/joystick_axe.cpp
      src/joystick_button.cpp)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    rclcpp_lifecycle
    romea_core_common
    romea_common_utils
    sensor_msgs)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} ${YAML_CPP_LIBRARIES})

ament_export_include_directories(include)
ament_export_dependencies(romea_core_common)
ament_export_dependencies(romea_common_utils)
ament_export_dependencies(sensor_msgs)
ament_export_libraries(${PROJECT_NAME})
#ament_export_targets(${PROJECT_NAME})

#install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(
TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)

endif()

ament_package()




