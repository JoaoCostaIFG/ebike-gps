#include "ebike-battery.hpp"
#include "ebike-log.hpp"

#define VOLTAGE_DIVIDER_RATIO 2.0
#define MAX_BATTERY_V 4200
#define MIN_BATTERY_V 3600

double readBattery()
{
    uint32_t rawAdc = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    uint32_t voltage = rawAdc * VOLTAGE_DIVIDER_RATIO;

    double battery_percentage = ((double)(voltage - MIN_BATTERY_V) / (MAX_BATTERY_V - MIN_BATTERY_V)) * 100.0;
    if (battery_percentage > 100.0){
        battery_percentage = 100.0;
    }
    if (battery_percentage < 0.0){
        battery_percentage = 0.0;
    }

    return battery_percentage;
}