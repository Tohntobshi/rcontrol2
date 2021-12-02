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

    float currentHeightError,
    float heightErrorChangeRate,

    float currentYawError,
    float yawErrorChangeRate,
    
    int32_t frontLeft,
    int32_t frontRight,
    int32_t backLeft,
    int32_t backRight,

    float pidLoopFreq,
    
    float pitchErrInt,
    float rollErrInt,
    float yawErrInt,
    float heightErrInt
)
{

    uint32_t size = 1 + 17 * 4;
    uint32_t tmp1 = htonl(*(uint32_t *)(&currentPitchError));
    uint32_t tmp2 = htonl(*(uint32_t *)(&currentRollError));

    uint32_t tmp3 = htonl(*(uint32_t *)(&pitchErrorChangeRate));
    uint32_t tmp4 = htonl(*(uint32_t *)(&rollErrorChangeRate));

    uint32_t tmp5 = htonl(*(uint32_t *)(&currentHeightError));
    uint32_t tmp6 = htonl(*(uint32_t *)(&heightErrorChangeRate));

    uint32_t tmp7 = htonl(*(uint32_t *)(&currentYawError));
    uint32_t tmp8 = htonl(*(uint32_t *)(&yawErrorChangeRate));

    uint32_t tmp9 = htonl(frontLeft);
    uint32_t tmp10 = htonl(frontRight);
    uint32_t tmp11 = htonl(backLeft);
    uint32_t tmp12 = htonl(backRight);

    uint32_t tmp13 = htonl(*(uint32_t *)(&pidLoopFreq));

    uint32_t tmp14 = htonl(*(uint32_t *)(&pitchErrInt));
    uint32_t tmp15 = htonl(*(uint32_t *)(&rollErrInt));
    uint32_t tmp16 = htonl(*(uint32_t *)(&yawErrInt));
    uint32_t tmp17 = htonl(*(uint32_t *)(&heightErrInt));

    uint8_t * data = new uint8_t[size];
    data[0] = MessageTypes::INFO;
    memcpy(data + 1, &tmp1, 4);
    memcpy(data + 5, &tmp2, 4);
    memcpy(data + 9, &tmp3, 4);
    memcpy(data + 13, &tmp4, 4);
    memcpy(data + 17, &tmp5, 4);
    memcpy(data + 21, &tmp6, 4);
    memcpy(data + 25, &tmp7, 4);
    memcpy(data + 29, &tmp8, 4);
    memcpy(data + 33, &tmp9, 4);
    memcpy(data + 37, &tmp10, 4);
    memcpy(data + 41, &tmp11, 4);
    memcpy(data + 45, &tmp12, 4);
    memcpy(data + 49, &tmp13, 4);
    memcpy(data + 53, &tmp14, 4);
    memcpy(data + 57, &tmp15, 4);
    memcpy(data + 61, &tmp16, 4);
    memcpy(data + 65, &tmp17, 4);
    connection->enqueueToSend({ .data = data, .size = size, .ignoreWithoutConnection = true });
}