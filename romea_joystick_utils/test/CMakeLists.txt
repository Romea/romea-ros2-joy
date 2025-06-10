find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(${PROJECT_NAME}_button_test button_test.cpp)
target_link_libraries(${PROJECT_NAME}_button_test ${PROJECT_NAME})

ament_add_gtest(${PROJECT_NAME}_joystick_test joystick_test.cpp)
target_link_libraries(${PROJECT_NAME}_joystick_test ${PROJECT_NAME} yaml-cpp)

find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(${PROJECT_NAME}_remapping_test remapping_test.py)