#pragma once

/*
// Replace with your phone number (to send SMS commands)
#define MY_PHONE "+ZZXXXXXX"
// URL of the traccar server
#define TRACCAR_URL "https://traccar.org"
// ID of the device in the traccar server
#define TRACCAR_ID "123"
// It depends on the operator whether to set up an APN
#define NETWORK_APN "internet"
// SIM might need PIN to unlock (optional)
#define SIMCARD_PIN "1234"
*/
#include "ebike-conf-priv.h"

//#define EBIKE_DEBUG_BUILD

// See all AT commands
// #define DUMP_AT_COMMANDS

// Set RX buffer to 1Kb
#define TINY_GSM_RX_BUFFER 1024
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// How many seconds to wait between location reports
#define REPORT_LOCATION_RATE_SECOND 20
// Enable traccar from the start
#define TRACCAR_ENABLED false
// The GPIO pin for accelerometer interrupt
#define ACCEL_PIN GPIO_NUM_32
// How long to go without detecting movements before enabling alarm (in ms)
#define ALARM_ENABLE_TIMEOUT (2 * 60 * 1000) // 2m
// How long to go without detecting movements before shutting off (in ms)
#define DEEP_SLEEP_TIMEOUT (12 * 60 * 60 * 1000) // 12h