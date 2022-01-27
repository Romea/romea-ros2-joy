find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_trigger_test trigger_test.cpp )
target_link_libraries(${PROJECT_NAME}_trigger_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_stick_test stick_test.cpp )
target_link_libraries(${PROJECT_NAME}_stick_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_button_test button_test.cpp )
target_link_libraries(${PROJECT_NAME}_button_test ${PROJECT_NAME})

#ament_add_gtest(${PROJECT_NAME}_mapping_test mapping_test.cpp )
#target_link_libraries(${PROJECT_NAME}_mapping_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_remapping_test remapping_test.cpp )
target_link_libraries(${PROJECT_NAME}_remapping_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_joystick_test joystick_test.cpp )
target_link_libraries(${PROJECT_NAME}_joystick_test ${PROJECT_NAME})
