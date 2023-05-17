
set(SOURCES master2.c)
add_executable(master2 ${SOURCES})
target_link_libraries(master2 soem)
target_link_libraries(master2 m)
install(TARGETS master2 DESTINATION bin)







