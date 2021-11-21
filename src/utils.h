#pragma once
#include "stdint.h"

namespace Utils {
    float getFloatFromNet(uint8_t * data);
    int getIntFromNet(uint8_t * data);
}