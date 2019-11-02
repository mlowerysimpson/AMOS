cmake_minimum_required(VERSION 2.8)
project(RemoteControlTest)
# Locate libraries and headers
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")


find_package(Threads REQUIRED)
find_library(wiringPi_LIB wiringPi)
find_package(libgps REQUIRED)
find_package(CURL REQUIRED)
find_package( OpenCV REQUIRED )

# Include headers
include_directories(${WIRINGPI_INCLUDE_DIRS})
include_directories(${LIBGPS_INCLUDE_DIR})
include_directories(${CURL_INCLUDE_DIR})

#the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "./*.cpp")
add_executable(RemoteControlTest ${SOURCES})

# Link against libraries
target_link_libraries(RemoteControlTest ${CMAKE_THREAD_LIBS_INIT})
target_link_libraries(RemoteControlTest ${wiringPi_LIB} )
target_link_libraries(RemoteControlTest ${LIBGPS_LIBRARIES} )
target_link_libraries(RemoteControlTest ${CURL_LIBRARIES})
target_link_libraries( RemoteControlTest ${OpenCV_LIBS} )

