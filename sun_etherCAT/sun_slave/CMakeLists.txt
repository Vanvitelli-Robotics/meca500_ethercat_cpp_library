project (sun_slave)

file(GLOB ${PROJECT_NAME}_SOURCES src/*.cpp)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)

target_link_libraries(${PROJECT_NAME} sun_ethercat_master)

