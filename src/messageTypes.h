#pragma once
#include <cstdint>

enum MessageTypes : uint8_t
{
	CONTROLS,
	INFO
};

enum Controls : uint8_t
{
	SET_PITCH_AND_ROLL
};