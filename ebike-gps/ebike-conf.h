#pragma once

#include "ebike-conf-priv.h"

#define EBIKE_DEBUG_BUILD

// See all AT commands
// #define DUMP_AT_COMMANDS

// Set RX buffer to 1Kb
#define TINY_GSM_RX_BUFFER 1024
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// It depends on the operator whether to set up an APN
// #define NETWORK_APN "internet"
// SIM might need PIN to unlock
// #define SIMCARD_PIN "1234"