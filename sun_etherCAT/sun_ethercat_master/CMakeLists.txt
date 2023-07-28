project (sun_ethercat_master)

#add_compile_options(-pthread)

set(${PROJECT_NAME}_SOURCES src/Master.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} soem)
target_link_libraries(${PROJECT_NAME} sun_scheduling)

