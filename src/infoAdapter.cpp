#include "infoAdapter.h"
#include "messageTypes.h"
#include <cstring>

InfoAdapter::InfoAdapter(Connection * conn)
: connection(conn)
{
}

void InfoAdapter::sendInfo(
    float currentPitchError,
    float currentRollError,
    float pitchErrorChangeRate,
    float rollErrorChangeRate,
    float currentYawSpeedError,
    float yawSpeedErrorChangeRate
)
{
    // printf("err %f %f\n", currentPitchError, currentRollError);

    uint32_t size = 25;
    uint32_t tmp1 = htonl(*(uint32_t *)(&currentPitchError));
    uint32_t tmp2 = htonl(*(uint32_t *)(&currentRollError));
    uint32_t tmp3 = htonl(*(uint32_t *)(&pitchErrorChangeRate));
    uint32_t tmp4 = htonl(*(uint32_t *)(&rollErrorChangeRate));
    uint32_t tmp5 = htonl(*(uint32_t *)(&currentYawSpeedError));
    uint32_t tmp6 = htonl(*(uint32_t *)(&yawSpeedErrorChangeRate));

    uint8_t * data = new uint8_t[size];
    data[0] = MessageTypes::INFO;
    memcpy(data + 1, &tmp1, 4);
    memcpy(data + 5, &tmp2, 4);
    memcpy(data + 9, &tmp3, 4);
    memcpy(data + 13, &tmp4, 4);
    memcpy(data + 17, &tmp5, 4);
    memcpy(data + 21, &tmp6, 4);
    connection->enqueueToSend({ .data = data, .size = size }, true);
}