cmake_minimum_required(VERSION 2.8.12)
project (meca500_driver)

file(GLOB ${PROJECT_NAME}_SOURCES Robot.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} sun_ethercat_master sun_slave sun_controller)

# When done tweaking common stuff, configure the components (subprojects).
# NOTE: The order matters! The most independent ones should go first.
#add_subdirectory(SOEM)
#add_subdirectory(master) 

add_subdirectory(sun_etherCAT/sun_ethercat_master)
add_subdirectory(sun_etherCAT/SOEM)
add_subdirectory(sun_etherCAT/sun_scheduling)
add_subdirectory(sun_etherCAT/sun_slave)
add_subdirectory(sun_etherCAT/sun_controller)

add_executable(robot-test test.cpp)

target_link_libraries(robot-test meca500_driver)