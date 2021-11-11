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
    int32_t frontLeft,
    int32_t frontRight,
    int32_t backLeft,
    int32_t backRight
)
{
    // printf("err %f %f\n", currentPitchError, currentRollError);

    uint32_t size = 25 + 16;
    uint32_t tmp1 = htonl(*(uint32_t *)(&currentPitchError));
    uint32_t tmp2 = htonl(*(uint32_t *)(&currentRollError));
    uint32_t tmp3 = htonl(*(uint32_t *)(&pitchErrorChangeRate));
    uint32_t tmp4 = htonl(*(uint32_t *)(&rollErrorChangeRate));
    uint32_t tmp5 = htonl(*(uint32_t *)(&currentHeightError));
    uint32_t tmp6 = htonl(*(uint32_t *)(&heightErrorChangeRate));

    uint32_t tmp7 = htonl(frontLeft);
    uint32_t tmp8 = htonl(frontRight);
    uint32_t tmp9 = htonl(backLeft);
    uint32_t tmp10 = htonl(backRight);

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
    connection->enqueueToSend({ .data = data, .size = size }, true);
}