#pragma once
#include "connection.h"

class InfoAdapter
{
private:
    Connection * connection = nullptr;
public:
    InfoAdapter(Connection * conn);
    void sendInfo(
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
        );
};
