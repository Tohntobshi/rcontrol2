cmake_minimum_required(VERSION 2.8)
project( Flying )

include_directories(/usr/include/dbus-1.0)
include_directories(/usr/lib/x86_64-linux-gnu/dbus-1.0/include)
include_directories(/usr/lib/arm-linux-gnueabihf/dbus-1.0/include)

include_directories(libs/glm/include)
include_directories(libs/bmp280/include)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(
  TARGETS
    src/main.cpp
    src/connection.cpp
    src/bluetoothHelper.cpp
    src/controlsAdapter.cpp
    src/flightController.cpp
    src/infoAdapter.cpp
    src/utils.cpp
    src/videoEncoder.cpp
)

add_executable( rcontrol ${TARGETS} )

add_subdirectory( libs/bmp280 )

target_link_libraries(rcontrol bluetooth)
target_link_libraries(rcontrol dbus-1)
target_link_libraries(rcontrol pthread)
target_link_libraries(rcontrol pigpio)
target_link_libraries(rcontrol bmp280)
target_link_libraries(rcontrol ${OpenCV_LIBS})
target_link_libraries(rcontrol avcodec)
target_link_libraries(rcontrol avutil)
target_link_libraries(rcontrol swscale)