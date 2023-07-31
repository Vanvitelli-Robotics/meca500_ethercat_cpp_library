#cmake_minimum_required(VERSION 2.8.12)
cmake_minimum_required(VERSION 3.1.12)
project (meca500_driver)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

file(GLOB ${PROJECT_NAME}_SOURCES *.cpp)

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

add_subdirectory(sun_etherCAT/sun_ethercat_master)
add_subdirectory(sun_etherCAT/SOEM)
add_subdirectory(sun_etherCAT/sun_scheduling)
add_subdirectory(sun_etherCAT/sun_slave)
add_subdirectory(sun_etherCAT/sun_controller)

add_executable(robot-test test.cpp)
add_executable(matrix-test matrix-test.cpp)

target_link_libraries(${PROJECT_NAME} sun_ethercat_master sun_slave sun_controller)

target_link_libraries(robot-test meca500_driver)
target_link_libraries(matrix-test meca500_driver)