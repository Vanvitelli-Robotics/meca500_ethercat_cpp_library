cmake_minimum_required(VERSION 2.8.12)
project(global_project)

# When done tweaking common stuff, configure the components (subprojects).
# NOTE: The order matters! The most independent ones should go first.
#add_subdirectory(SOEM)
#add_subdirectory(master) 

add_subdirectory(sun_ethercat_master)
add_subdirectory(SOEM)
add_subdirectory(sun_scheduling)
add_subdirectory(sun_slave)
add_subdirectory(sun_controller)
#add_subdirectory(sottotest)
#add_executable(main_master src/main_master.cpp)
add_executable(main_position_control src/main_position_control.cpp)


#target_link_libraries(main_master sun_ethercat_master)
#target_link_libraries(main_master sun_slave)

target_link_libraries(main_position_control sun_ethercat_master)
target_link_libraries(main_position_control sun_slave)
target_link_libraries(main_position_control sun_controller)












