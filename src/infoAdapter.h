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
        float currentYawSpeedError,
        float yawSpeedErrorChangeRate
        );
};
