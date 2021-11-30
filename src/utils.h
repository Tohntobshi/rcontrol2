#pragma once
#include "stdint.h"
#include <array>

namespace Utils {
    float getFloatFromNet(uint8_t * data);
    int getIntFromNet(uint8_t * data);
    float trimAngleTo360(float angle);
    std::array<float, 2> pepareAnglesForCombination(float angle1, float angle2);
}