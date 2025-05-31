#pragma once

#include <memory>
#include <utility>
#include <sstream>

#include "ebike-conf.h"
#include "modem_utilities.h"

#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

#include "ebike-log.hpp"

// Docs: https://www.traccar.org/osmand/
#define POST_FORMAT "deviceid=%s&lat=%.7f&lon=%.7f&speed=%.2f&altitude=%.2f&accuracy=%.2f&hdop=%.2f&batt=%.2f&charge=%s"
// User Equivalent Range Error (UERE). Typically 5.0 for consumer grade GPS devices.
#define UERE 5.0

class EBikeGPS
{
private:
    std::shared_ptr<TinyGsm> modem;
    TinyGPSPlus gps;

    uint32_t gps_failures = 0;
    double accuracy_value = 0; // in meters

    double lat_value = 0;
    double lon_value = 0;
    double altitude_value = 0;
    uint32_t satellites_value = 0;

    double speed = 0; // m/s

    double hdop_value = 0; // Horizontal Dilution of Precision

    uint8_t day_value;
    uint8_t month_value;
    uint16_t year_value;

    uint8_t hour_value;
    uint8_t minute_value;
    uint8_t second_value;

public:
    EBikeGPS(std::shared_ptr<TinyGsm> modem)
    {
        this->modem = modem;
    }
    ~EBikeGPS() {}

    double accuracy()
    {
        return this->accuracy_value;
    }

    double lat()
    {
        return this->lat_value;
    }
    double lon()
    {
        return this->lon_value;
    }

    double altitude()
    {
        return this->altitude_value;
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

    double hdop()
    {
        return this->hdop_value;
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

    /*
     * Mode can be:
     * 1 - GPS L1 + SBAS + QZSS
     * 2 - BDS B1
     * 3 - GPS + GLONASS + GALILEO + SBAS + QZSS
     * 4 - GPS + BDS + GALILEO + SBAS + QZSS.
     */
    void enable(uint8_t mode)
    {
        EBIKE_DBG("Enabling GPS");
        while (!modem->enableGPS(MODEM_GPS_ENABLE_GPIO, MODEM_GPS_ENABLE_LEVEL))
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

        this->accuracy_value = (double)accuracy;
        this->hdop_value = 0.0; // GSM location does not provide HDOP
        this->lat_value = (double)lat;
        this->lon_value = (double)lon;
        this->year_value = (uint16_t)year;
        this->month_value = (uint8_t)month;
        this->day_value = (uint8_t)day;
        this->hour_value = (uint8_t)hour;
        this->minute_value = (uint8_t)min;
        this->second_value = (uint8_t)sec;
    }

    bool update()
    {
        GPSInfo info;
        bool rlst = modem->getGPS_Ex(info);
        if (!rlst)
        {
            this->gps_failures++;
            if (this->gps_failures > 10)
            {
                EBIKE_ERR("Failed to get GPS info, using GSM location as fallback.");
                this->bootstapWithGsm();
            }
            return false;
        }
        this->gps_failures = 0;

        this->accuracy_value = info.PDOP * UERE;
        this->altitude_value = info.altitude;
        this->lat_value = info.latitude;
        this->lon_value = info.longitude;
        this->satellites_value = info.gps_satellite_num + info.beidou_satellite_num +
                                 info.glonass_satellite_num + info.galileo_satellite_num;
        this->speed = info.speed;     // m/s
        this->hdop_value = info.HDOP; // Horizontal Dilution of Precision
        this->day_value = info.day;
        this->month_value = info.month;
        this->year_value = info.year;
        this->hour_value = info.hour;
        this->minute_value = info.minute;
        this->second_value = info.second;

        return true;
    }

    void display()
    {
        EBIKE_NFOF("Location (Lat,Lon): %.7f,%.7f", this->lat(), this->lon());
        EBIKE_NFO("Satelites: ", this->satellites());
        EBIKE_NFOF("Date: %04u-%02u-%02u", this->year(), this->month(), this->day());
        EBIKE_NFOF("Time: %02u:%02u:%02u", this->hour(), this->minute(), this->second());
        EBIKE_NFOF("Speed (km/h): %.2f", this->speed_kmph());
        EBIKE_NFO("--------------------------------");
    }

    bool post_location(const char *server_url, const char *client_id, double battery = 100)
    {
        bool ret = false;
        char post_buffer[256];
        snprintf(post_buffer, sizeof(post_buffer), POST_FORMAT,
                 client_id, this->lat(), this->lon(), this->speed_mps(),
                 this->altitude(), this->accuracy(), this->hdop(),
                 battery, (battery == 0.0) ? "true" : "false");

        // Initialize HTTPS
        modem->https_begin();

        // Set Post URL
        if (!modem->https_set_url(server_url))
        {
            EBIKE_ERR("Failed to set the URL. Please check the validity of the URL!");
            ret = false;
            return false;
        }
        else
        {
            modem->https_set_user_agent("TinyGSM/LilyGo-A7670");
            int httpCode = modem->https_post(post_buffer);
            if (httpCode == 200)
            {
                ret = true;
            }
            else
            {
                EBIKE_ERR("HTTP post failed! error code = ", httpCode);
                ret = false;
            }

            modem->https_end();
        }

        return ret;
    }
};
