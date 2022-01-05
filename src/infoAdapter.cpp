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

    float voltage,

    float currentPosXError,
    float currentPosYError,
    float posXErrorChangeRate,
    float posYErrorChangeRate,
    float posXErrorInt,
    float posYErrorInt
)
{
    uint32_t size = 97;

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

    Utils::setFloatToNet(currentPosXError, data + 73);
    Utils::setFloatToNet(currentPosYError, data + 77);
    Utils::setFloatToNet(posXErrorChangeRate, data + 81);
    Utils::setFloatToNet(posYErrorChangeRate, data + 85);
    Utils::setFloatToNet(posXErrorInt, data + 89);
    Utils::setFloatToNet(posYErrorInt, data + 93);

    connection->enqueueToSend({ .data = data, .size = size, .ignoreWithoutConnection = true });
}

void InfoAdapter::sendPrimaryInfo(
    uint8_t landingFlag,
    float voltage,
    uint8_t positionValidity,
    int32_t satelitesAmount
)
{
    uint32_t size = 11;

    uint8_t * data = new uint8_t[size];
    data[0] = (uint8_t)MessageTypes::PRIMARY_INFO;
    data[1] = landingFlag;
    Utils::setFloatToNet(voltage, data + 2);
    data[6] = positionValidity;
    Utils::setIntToNet(satelitesAmount, data + 7);

    connection->enqueueToSend({ .data = data, .size = size, .ignoreWithoutConnection = true });
}