#include "ebike-wifi.hpp"

#include <Arduino.h>
#ifdef ARDUINO_ARCH_SAMD
#include <WiFi101.h>
#elif defined ARDUINO_ARCH_ESP8266
#include <ESP8266WiFi.h>
#elif defined ARDUINO_ARCH_ESP32
#include <WiFi.h>
#else
#error Wrong platform
#endif

#define MAC_ADDR_SIZE 18

static String MACtoString(uint8_t macAddress[6])
{
    char macStr[MAC_ADDR_SIZE] = {0};
    snprintf(macStr, MAC_ADDR_SIZE, "%02X:%02X:%02X:%02X:%02X:%02X", macAddress[0], macAddress[1], macAddress[2], macAddress[3], macAddress[4], macAddress[5]);
    return String(macStr);
}

String getSurroundingWiFiJson()
{
    String wifiArray = "[";

    int8_t numWifi = WiFi.scanNetworks();
    for (uint8_t i = 0; i < numWifi; i++)
    {
        // filter mac
        uint8_t *mac_addr = WiFi.BSSID(i);
        if (mac_addr[0] & 2)
        {
            // skip locally admininstered MAC
            // remove such MAC addresses by ensuring that the second least-significant
            // bit of the MAC's most-significant byte is 0
            continue;
        }
        else if (mac_addr[0] == 0x0 && mac_addr[1] == 0x0 && mac_addr[2] == 0x5E)
        {
            // The range of MAC addresses between 00:00:5E:00:00:00 and 00:00:5E:FF:FF:FF
            // are reserved for IANA and often used for network management and multicast
            // functions which precludes their use as a location signal.
            continue;
        }

        wifiArray += "{\"macAddress\":\"" + MACtoString(WiFi.BSSID(i)) + "\",";
        wifiArray += "\"signalStrength\":" + String(WiFi.RSSI(i)) + ",";
        wifiArray += "\"channel\":" + String(WiFi.channel(i)) + "}";
        if (i < (numWifi - 1))
        {
            wifiArray += ",\n";
        }
    }
    WiFi.scanDelete();
    wifiArray += "]";

    return wifiArray;
}