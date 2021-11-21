#include "utils.h"
#include <arpa/inet.h>

float Utils::getFloatFromNet(uint8_t * data) {
    uint32_t tmp = ntohl(*(uint32_t *)(data));
    return *(float *)(&tmp);
}

int Utils::getIntFromNet(uint8_t * data) {
    return ntohl(*(uint32_t*)(data));
}
