find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_trigger_test trigger_test.cpp )
target_link_libraries(${PROJECT_NAME}_trigger_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_stick_test stick_test.cpp )
target_link_libraries(${PROJECT_NAME}_stick_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_button_test button_test.cpp )
target_link_libraries(${PROJECT_NAME}_button_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_configuration_test configuration_test.cpp )
target_link_libraries(${PROJECT_NAME}_configuration_test ${PROJECT_NAME} yaml-cpp )

ament_add_gtest(${PROJECT_NAME}_remapping_test remapping_test.cpp )
target_link_libraries(${PROJECT_NAME}_remapping_test ${PROJECT_NAME} yaml-cpp )

ament_add_gtest(${PROJECT_NAME}_joystick_test joystick_test.cpp )
target_link_libraries(${PROJECT_NAME}_joystick_test ${PROJECT_NAME} yaml-cpp )
