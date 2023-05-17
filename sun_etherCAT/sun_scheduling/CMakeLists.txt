project (sun_scheduling)

set(${PROJECT_NAME}_SOURCES src/scheduling.c)
add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC include)
