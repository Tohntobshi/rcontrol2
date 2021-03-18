cmake_minimum_required(VERSION 2.8)
project( BMP280 )

include_directories( ./include/bmp280 )

set(
  TARGETS
    bmp280.c
)

add_library( bmp280 STATIC ${TARGETS} )



