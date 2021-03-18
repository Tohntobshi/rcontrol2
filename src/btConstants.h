#pragma once
#include <bluetooth/bluetooth.h>

static bdaddr_t BDADDR_ANY_VAL = {{0, 0, 0, 0, 0, 0}};
static bdaddr_t BDADDR_ALL_VAL =  {{0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
static bdaddr_t BDADDR_LOCAL_VAL = {{0, 0, 0, 0xff, 0xff, 0xff}};