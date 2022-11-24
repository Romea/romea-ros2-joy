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
find_package(romea_core_common REQUIRED)
find_package(romea_common_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
      src/joystick.cpp
      src/joystick_axe.cpp
      src/joystick_button.cpp
      src/joystick_configuration.cpp
      src/joystick_directional_pad.cpp
      src/joystick_remapping.cpp
      src/joystick_stick.cpp
      src/joystick_trigger.cpp
    )

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    romea_core_common
    romea_common_utils
    sensor_msgs
    yaml-cpp
    )

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

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)

    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
    add_subdirectory(test)

endif()

ament_package()





#cmake_minimum_required(VERSION 2.8.3)
#project(romea_common_utils)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Weffc++ -Wall -Wextra -std=c++17 -O3")

########################
### Find dependencies ##
########################
#find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros tf2_eigen nav_msgs diagnostic_updater)
#find_package(romea_common REQUIRED)

####################################
### catkin specific configuration ##
####################################
#catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})

############
### Build ##
############

##include_directories(
##    include
##    SYSTEM ${catkin_INCLUDE_DIRS}
##    SYSTEM ${EIGEN3_INCLUDE_DIRS}
##    SYSTEM ${romea_common_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}
#  src/conversions/DiagnosticConversions.cpp
#  src/conversions/TimeConversions.cpp
#  src/conversions/GeometryConversions.cpp
#  src/conversions/TransformConversions.cpp)

#target_include_directories( ${PROJECT_NAME} PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
#  )

#target_include_directories( ${PROJECT_NAME} SYSTEM PUBLIC
#  ${catkin_INCLUDE_DIRS}
#  ${romea_common_INCLUDE_DIRS}
#  )

#target_link_libraries(${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#  romea_common::romea_common)


##############
### Install ##
##############

## all install targets should use catkin DESTINATION variables
## See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

### Mark executable scripts (Python etc.) for installation
### in contrast to setup.py, you can choose the destination
## install(PROGRAMS
##   scripts/my_python_script
##   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
## )

### Mark executables and/or libraries for installation
##install(TARGETS romea
##    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
##    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
##    )

### Mark cpp header files for installation
##install(DIRECTORY include/${PROJECT_NAME}/
##    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
##    FILES_MATCHING PATTERN "*.hpp"
##    PATTERN ".svn" EXCLUDE
##    )

### Mark other files for installation (e.g. launch and bag files, etc.)
## install(FILES
##   # myfile1
##   # myfile2
##   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
## )

##############
### Testing ##
##############

#if(CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif(CATKIN_ENABLE_TESTING)


#cmake_minimum_required(VERSION 2.8.3)
#project(romea_joy)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=c++17")

#########################
#### Find dependencies ##
#########################
#set(${PROJECT_NAME}_CATKIN_DEPS roscpp sensor_msgs romea_common_utils)

#find_package(catkin COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})
#find_package(romea_common REQUIRED)

####################################
### catkin specific configuration ##
####################################
#catkin_package(
#    INCLUDE_DIRS include
#    LIBRARIES ${PROJECT_NAME}
#    CATKIN_DEPENDS ${${PROJECT_NAME}_CATKIN_DEPS})


#############
#### Build ##
#############
#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${romea_common_INCLUDE_DIRS})

#add_library(${PROJECT_NAME}
#  src/joystick.cpp
#  src/joystick_axis.cpp
#  src/joystick_button.cpp
#  src/joystick_directional_pad.cpp
#  src/joystick_mapping.cpp
#  src/joystick_stick.cpp
#  src/joystick_trigger.cpp
#  )

#target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

#if(CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif(CATKIN_ENABLE_TESTING)
