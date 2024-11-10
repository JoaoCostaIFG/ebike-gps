#pragma once

#include <memory>
#include <utility>

#include "ebike-conf.h"
#include "modem_utilities.h"

#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

#include "ebike-log.hpp"
#include "ebike-wifi.hpp"

#define GOOGLE_GEOLOCATION_API_URL "https://www.googleapis.com/geolocation/v1/geolocate?key="

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

    double speed = 0; // m/s

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

    int queryGoogleForLocation(std::pair<double, double> &coords)
    {
        String wifis = getSurroundingWiFiJson();

        if (!modem->https_set_url(GOOGLE_GEOLOCATION_API_URL MAPS_API_KEY))
        {
            EBIKE_ERR("Failed to set URL");
            return 1;
        }
        modem->https_add_header("Connection", "keep-alive");
        modem->https_set_accept_type("application/json");
        modem->https_set_user_agent("TinyGSM/A7670");
        String post_body = "{\"considerIP\":false,\"wifiAccessPoints\":" + wifis + "}";

        int httpCode = modem->https_post(post_body);
        if (httpCode != 200)
        {
            EBIKE_ERR("HTTP post failed ! error code = ", httpCode);
            return 1;
        }

        String body = modem->https_body();
        EBIKE_DBG("HTTP body : ", body);
        int idx = body.indexOf("\"lat\":");
        if (idx != -1)
        {
            String tmp = body.substring(idx + 6);
            coords.first = tmp.toDouble();
        }
        else
        {
            EBIKE_ERR("Failed to get latitude from Google Geo Location.");
            return 2;
        }
        idx = body.indexOf("\"lng\":");
        if (idx != -1)
        {
            String tmp = body.substring(idx + 6);
            coords.second = tmp.toDouble();
        }
        else
        {
            EBIKE_ERR("Failed to get longitude from Google Geo Location.");
            return 3;
        }

        return 0;
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

    void bootstapWithGsm()
    {
        float lat = 0;
        float lon = 0;
        float accuracy = 0;
        int year = 0;
        int month = 0;
        int day = 0;
        int hour = 0;
        int min = 0;
        int sec = 0;
        EBIKE_DBG("Bootstraping location with GSM.");
        while (!modem->getGsmLocation(&lat, &lon, &accuracy, &year, &month, &day, &hour,
                                      &min, &sec))
        {
            EBIKE_DBG("Couldn't get GSM location, retrying...");
        }

        this->lat_value = (double)lat;
        this->lon_value = (double)lon;
        this->year_value = (uint16_t)year;
        this->month_value = (uint8_t)month;
        this->day_value = (uint8_t)day;
        this->hour_value = (uint8_t)hour;
        this->minute_value = (uint8_t)min;
        this->second_value = (uint8_t)sec;
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
            if (gps.location.isValid())
            {
                this->lat_value = gps.location.lat();
                this->lon_value = gps.location.lng();
            }
        }
        if (gps.satellites.isUpdated())
        {
            this->satellites_value = gps.satellites.value();
        }
        if (gps.speed.isUpdated())
        {
            this->speed = gps.speed.mps();
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
        // EBIKE_NFOF("Location (Lat,Lon): %.8f,%.8f\nhttps://www.google.com/maps/search/%.8f,%.8f", this->lat(), this->lon(), this->lat(), this->lon());
        EBIKE_NFOF("Location (Lat,Lon): %.8f,%.8f", this->lat(), this->lon());
        EBIKE_NFO("Satelites: ", this->satellites());
        EBIKE_NFOF("Date: %04u-%02u-%02u", this->year(), this->month(), this->day());
        EBIKE_NFOF("Time: %02u:%02u:%02u", this->hour(), this->minute(), this->second());
        EBIKE_NFOF("Speed (km/h): %.2f", this->speed_kmph());
        EBIKE_NFO("--------------------------------");
    }
};
