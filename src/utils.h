#pragma once
#include "stdint.h"
#include <array>

namespace Utils {
    float getFloatFromNet(uint8_t * data);
    int getIntFromNet(uint8_t * data);
    uint16_t getShortFromNet(uint8_t * data);
    void setFloatToNet(float val, uint8_t * dest);
    void setIntToNet(int val, uint8_t * dest);

    float trimAngleTo360(float angle);
    std::array<float, 2> pepareAnglesForCombination(float angle1, float angle2);
}