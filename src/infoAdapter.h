#pragma once
#include "connection.h"

class InfoAdapter
{
private:
    Connection * connection = nullptr;
public:
    InfoAdapter(Connection * conn);
    void sendSecondaryInfo(
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
        );
    void sendPrimaryInfo(
        uint8_t landingFlag,
        float voltage,
        uint8_t positionValidity,
        int32_t satelitesAmount
    );
    void sendVideoFrame(const uint8_t * data, int size);
};
