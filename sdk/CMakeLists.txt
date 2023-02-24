cmake_minimum_required(VERSION 2.8.3)

project(ord_sdk C CXX)

#set(CMAKE_BUILD_TYPE "Debug")

#if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
#  add_compile_options(-std=c++11)
#endif()

#c++ 11
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SDK_SRC_DIR "src")
set(SDK_SRC
  "${SDK_SRC_DIR}/ord_lidar_driver.cpp"
  "${SDK_SRC_DIR}/ord_lidar_sdk.cpp"
  "${SDK_SRC_DIR}/serial.cpp"
  "${SDK_SRC_DIR}/parse.cpp"
)
SET(CMAKE_INSTALL_PREFIX <release>)

add_library(${PROJECT_NAME} STATIC ${SDK_SRC})
set_target_properties(${PROJECT_NAME} PROPERTIES PREFIX "")

target_include_directories(${PROJECT_NAME}
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  set(SDK_LIB_DEPS pthread)
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
  set(SDK_LIB_DEPS setupapi Winmm)
endif()

target_link_libraries(${PROJECT_NAME} ${SDK_LIB_DEPS})

add_executable(blocking_test samples/blocking_test.cpp)
target_include_directories(blocking_test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(blocking_test ord_sdk)

add_executable(non-blocking_test samples/non-blocking_test.cpp)
target_include_directories(non-blocking_test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(non-blocking_test ord_sdk)

add_executable(blocking_c_api_test samples/blocking_c_api_test.c)
target_include_directories(blocking_c_api_test PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(blocking_c_api_test ord_sdk)

