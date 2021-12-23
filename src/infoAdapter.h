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
        float heightErrInt
        );
    void sendPrimaryInfo(
        uint8_t landingFlag
    );
};
