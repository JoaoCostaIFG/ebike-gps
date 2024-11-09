#pragma once

#include <memory>

#include "ebike-conf.h"
#include "modem_utilities.h"

#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

#include "ebike-log.hpp"

enum GPS_RATE
{
    GPS_1HZ = 1,
    GPS_2HZ = 2,
    GPS_5HZ = 5,
};

class EBikeGPS
{
private:
    std::shared_ptr<TinyGsm> modem;
    TinyGPSPlus gps;

    double lat_value = 0;
    double lon_value = 0;
    uint32_t satellites_value = 0;

    double speed = 0;    // m/s
    double altitude = 0; // m

    uint8_t day_value;
    uint8_t month_value;
    uint16_t year_value;

    uint8_t hour_value;
    uint8_t minute_value;
    uint8_t second_value;

    void consume()
    {
        while (SerialAT.available())
        {
            int ch = SerialAT.read();
#ifdef DUMP_AT_COMMANDS
            Serial.write(ch);
#endif // DUMP_AT_COMMANDS
            gps.encode(ch);
        }
    }

public:
    EBikeGPS(std::shared_ptr<TinyGsm> modem)
    {
        this->modem = modem;
    }
    ~EBikeGPS() {}

    double lat()
    {
        return this->lat_value;
    }
    double lon()
    {
        return this->lon_value;
    }

    uint32_t satellites()
    {
        return this->satellites_value;
    }

    double speed_kmph()
    {
        return this->speed * 3.6;
    }
    double speed_mps()
    {
        return this->speed;
    }

    double altitude_m()
    {
        return this->altitude;
    }
    double altitude_km()
    {
        return this->altitude / 1000;
    }

    uint8_t day()
    {
        return this->day_value;
    }
    uint8_t month()
    {
        return this->month_value;
    }
    uint16_t year()
    {
        return this->year_value;
    }

    uint8_t hour()
    {
        return this->hour_value;
    }
    uint8_t minute()
    {
        return this->minute_value;
    }
    uint8_t second()
    {
        return this->second_value;
    }

    void enable(uint8_t mode, enum GPS_RATE rate)
    {
        EBIKE_DBG("Enabling GPS");
        while (!modem->enableGPS(MODEM_GPS_ENABLE_GPIO))
        {
            EBIKE_DBG(".");
            sleep(100);
        }
        EBIKE_NFO("GPS Enabled");
        modem->setGPSBaud(MODEM_BAUDRATE);
        // GPS+GLONASS+GALILEO+SBAS+QZSS
        if (!modem->setGPSMode(mode))
        {
            EBIKE_ERR("Failed to set gps mode");
        }
        modem->configNMEASentence(1, 1, 1, 1, 1, 1);
        modem->setGPSOutputRate(rate);
        modem->enableNMEA();
        if (!modem->enableAGPS())
        {
            EBIKE_ERR("Failed to enable AGPS");
        }
    }

    /**
     * This custom version of delay() ensures that the gps object
     * is being "fed".
     */
    void delay(unsigned long ms)
    {
        unsigned long start = millis();
        if (start > 30000UL && gps.charsProcessed() < 10U)
        {
            EBIKE_ERR("No GPS data received yet: check wiring");
        }

        // TODO not here
        // restart GPS is needed
        if (!modem->isEnableGPS())
        {
            EBIKE_DBG("Restart GPS!");
            modem->enableGPS();
        }

        this->consume();

        if (gps.location.isUpdated())
        {
            this->lat_value = gps.location.lat();
            this->lon_value = gps.location.lng();
        }
        if (gps.satellites.isUpdated())
        {
            this->satellites_value = gps.satellites.value();
        }
        if (gps.speed.isUpdated())
        {
            this->speed = gps.speed.mps();
        }
        if (gps.altitude.isUpdated())
        {
            gps.altitude.meters();
        }
        if (gps.date.isUpdated())
        {
            this->day_value = gps.date.day();
            this->month_value = gps.date.month();
            this->year_value = gps.date.year();
        }
        if (gps.time.isUpdated())
        {
            this->hour_value = gps.time.hour();
            this->minute_value = gps.time.minute();
            this->second_value = gps.time.second();
        }

        // wait remaining requested time
        do
        {
            this->consume();
        } while (millis() - start < ms);
    }

    void display()
    {
        EBIKE_NFOF("Location: Lat=%.6f Lon=%.6f", this->lat(), this->lon());
        EBIKE_NFO("Satelites: ", this->satellites());
        EBIKE_NFOF("Date: %04u-%02u-%02u", this->year(), this->month(), this->day());
        EBIKE_NFOF("Time: %02u:%02u:%02u", this->hour(), this->minute(), this->second());
        EBIKE_NFOF("Speed (km/h): %.2f", this->speed_kmph());
        EBIKE_NFOF("Altitude (m): %.2f", this->altitude_m());
        EBIKE_NFO("--------------------------------");
    }
};