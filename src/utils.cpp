#include "utils.h"
#include <arpa/inet.h>
#include <math.h>

float Utils::getFloatFromNet(uint8_t * data) {
    uint32_t tmp = ntohl(*(uint32_t *)(data));
    return *(float *)(&tmp);
}

int Utils::getIntFromNet(uint8_t * data) {
    return ntohl(*(uint32_t*)(data));
}

float Utils::trimAngleTo360(float angle) {
    if (angle < 0.f) {
        return trimAngleTo360(angle + 360.f);
    } else if (angle >= 360.f) {
        return trimAngleTo360(angle - 360.f);
    } else {
        return angle;
    }
}

std::array<float, 2> Utils::pepareAnglesForCombination(float angle1, float angle2) {
    angle1 = trimAngleTo360(angle1);
    angle2 = trimAngleTo360(angle2);
    if (abs(angle2 - angle1) <= 180.f) {
        return { angle1, angle2 };
    } else if (angle1 > angle2) {
        return { angle1, angle2 + 360.f };
    } else {
        return { angle1 + 360.f, angle2 };
    }
}