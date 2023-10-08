# meca500-ethercat-cpp

This is a library that simplifies the communication to a Meca500 robotic arm via Ethercat.

You can find an example of usage in the file test.cpp.

## How to include the library

If you are using cmake you can clone this repository in your project folder and add it as a subfolder in your CMakeLists.txt with this command:

```cmake
add_subdirectory(meca500_ethercat_cpp)
```

Then you can link the library to your target with this command:

```cmake
target_link_libraries(${EXECUTABLE_NAME} meca500_driver)
```
