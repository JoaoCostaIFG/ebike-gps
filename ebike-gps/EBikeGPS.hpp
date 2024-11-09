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

    // double speed = 0; // m/s

public:
    EBikeGPS(std::shared_ptr<TinyGsm> modem)
    {
        this->modem = modem;
    }
    ~EBikeGPS() {}

    double lat()
    {
        return gps.location.lat();
    }
    double lon()
    {
        return gps.location.lng();
    }

    uint32_t satellites()
    {
        return gps.satellites.value();
    }

    double speed_kmph()
    {
        return gps.speed.kmph();
    }
    double speed_mph()
    {
        return gps.speed.mph();
    }
    double speed_mps()
    {
        return gps.speed.mps();
    }
    double speed_knots()
    {
        return gps.speed.knots();
    }

    uint8_t day()
    {
        return gps.date.day();
    }
    uint8_t month()
    {
        return gps.date.month();
    }
    uint16_t year()
    {
        return gps.date.year();
    }

    uint8_t hour()
    {
        return gps.time.hour();
    }
    uint8_t minute()
    {
        return gps.time.minute();
    }
    uint8_t second()
    {
        return gps.time.second();
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

        do
        {
            while (SerialAT.available())
            {
                int ch = SerialAT.read();
                // Serial.write(ch);
                gps.encode(ch);
            }
        } while (millis() - start < ms);
    }

    void display()
    {
        EBIKE_NFOF("Location: Lat=%.6f Lon=%.6f", this->lat(), this->lon());
        EBIKE_NFO("Satelites: ", this->satellites());
        EBIKE_NFOF("Date: %04u-%02u-%02u", this->year(), this->month(), this->day());
        EBIKE_NFOF("Time: %02u:%02u:%02u", this->hour(), this->minute(), this->second());
        EBIKE_NFOF("Speed: %.2f", gps.speed.kmph());
        EBIKE_NFO("--------------------------------");
    }
};