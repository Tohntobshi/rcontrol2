#include "infoAdapter.h"
#include "messageTypes.h"
#include <cstring>
#include "utils.h"

InfoAdapter::InfoAdapter(Connection * conn)
: connection(conn)
{
}

void InfoAdapter::sendSecondaryInfo(
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
    float heightErrInt,

    float voltage
)
{
    uint32_t size = 73;

    uint8_t * data = new uint8_t[size];
    data[0] = (uint8_t)MessageTypes::SECONDARY_INFO;
    Utils::setFloatToNet(currentPitchError, data + 1);
    Utils::setFloatToNet(currentRollError, data + 5);
    Utils::setFloatToNet(pitchErrorChangeRate, data + 9);
    Utils::setFloatToNet(rollErrorChangeRate, data + 13);
    Utils::setFloatToNet(currentHeightError, data + 17);
    Utils::setFloatToNet(heightErrorChangeRate, data + 21);
    Utils::setFloatToNet(currentYawError, data + 25);
    Utils::setFloatToNet(yawErrorChangeRate, data + 29);
    Utils::setIntToNet(frontLeft, data + 33);
    Utils::setIntToNet(frontRight, data + 37);
    Utils::setIntToNet(backLeft, data + 41);
    Utils::setIntToNet(backRight, data + 45);
    Utils::setFloatToNet(pidLoopFreq, data + 49);
    Utils::setFloatToNet(pitchErrInt, data + 53);
    Utils::setFloatToNet(rollErrInt, data + 57);
    Utils::setFloatToNet(yawErrInt, data + 61);
    Utils::setFloatToNet(heightErrInt, data + 65);
    Utils::setFloatToNet(voltage, data + 69);

    connection->enqueueToSend({ .data = data, .size = size, .ignoreWithoutConnection = true });
}

void InfoAdapter::sendPrimaryInfo(
    uint8_t landingFlag,
    float voltage
)
{
    uint32_t size = 6;

    uint8_t * data = new uint8_t[size];
    data[0] = (uint8_t)MessageTypes::PRIMARY_INFO;
    data[1] = landingFlag;
    Utils::setFloatToNet(voltage, data + 2);

    connection->enqueueToSend({ .data = data, .size = size, .ignoreWithoutConnection = true });
}