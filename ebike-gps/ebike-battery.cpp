#include "ebike-battery.hpp"
#include "ebike-log.hpp"

#include <Arduino.h>
#include "modem_utilities.h"

#define VOLTAGE_DIVIDER_RATIO 2.0
#define MAX_BATTERY_V 4.2
#define MIN_BATTERY_V 3.0

double readBattery()
{
    uint32_t rawAdc = analogRead(BOARD_BAT_ADC_PIN);
    double voltage = ((double)rawAdc / 4095.0) * 3.3 * VOLTAGE_DIVIDER_RATIO;
    EBIKE_NFO("Raw ADC: ", rawAdc, " Voltage: ", voltage);

    double battery_percentage = ((voltage - MIN_BATTERY_V) / (MAX_BATTERY_V - MIN_BATTERY_V)) * 100.0;
    EBIKE_NFO("Battery percentage: ", battery_percentage);
    if (battery_percentage > 100.0){
        battery_percentage = 100.0;
    }
    if (battery_percentage < 0.0){
        battery_percentage = 0.0;
    }

    return battery_percentage;
}