#include "ebike-conf.h"

#include <Arduino.h>

#include "modem_utilities.h"
#include <TinyGsmClient.h>
#include <TinyGPSPlus.h>

#include "ebike-log.h"

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// Global data
static TinyGPSPlus gps;
static uint32_t check_interval = 0;

static void wait_modem_start()
{
  EBIKE_DBG("Starting modem...");
  delay(3000);
  for (unsigned retry = 0; !modem.testAT(1000); ++retry)
  {
    EBIKE_DBG(".");
    if (retry > 10)
    {
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
    }
  }
  EBIKE_DBG("Modem started.");
  delay(200);
}

static int wait_sim_online()
{
  SimStatus sim = SIM_ERROR;
  while (sim != SIM_READY)
  {
    sim = modem.getSimStatus();
    switch (sim)
    {
    case SIM_READY:
      EBIKE_DBG("SIM online.");
      break;
    case SIM_LOCKED:
#ifdef SIMCARD_PIN
      if (!modem.simUnlock(SIMCARD_PIN))
      {
        EBIKE_ERR("Failed to unlock SIM card");
        return 1;
      }
#else
      EBIKE_ERR("SIM locked: needs PIN.");
      return 1;
#endif // SIMCARD_PIN
      break;
    default:
      break;
    }
    delay(1000);
  }

  return 0;
}

/**
 * Check network registration status and network signal status
 */
static int wait_network_registration()
{
  EBIKE_DBG("Wait for the modem to register with the network.");
  RegStatus status = REG_NO_RESULT;
  while (status == REG_NO_RESULT || status == REG_SEARCHING ||
         status == REG_UNREGISTERED)
  {

    status = modem.getRegistrationStatus();
    switch (status)
    {
    case REG_UNREGISTERED:
    case REG_SEARCHING:
      EBIKE_DBG("Searching signal quality: ", modem.getSignalQuality());
      delay(1000);
      break;
    case REG_DENIED:
      EBIKE_ERR("Network registration was rejected, please check if the "
                "APN is correct.");
      return 1;
    case REG_OK_HOME:
      EBIKE_DBG("Online registration successful.");
      break;
    case REG_OK_ROAMING:
      EBIKE_DBG(
          "Network registration successful, currently in roaming mode.");
      break;
    default:
      EBIKE_DBG("Registration Status: %d\n", status);
      delay(1000);
      break;
    }
  }

  delay(1000);

  String ueInfo;
  if (modem.getSystemInformation(ueInfo))
  {
    EBIKE_DBG("Inquiring UE system information: ", ueInfo);
  }

  return 0;
}

static void enable_gps()
{
  EBIKE_DBG("Enabling GPS+GLONASS+GALILEO+SBAS+QZSS");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO))
  {
    EBIKE_DBG(".");
    sleep(100);
  }
  EBIKE_NFO("GPS Enabled");
  modem.setGPSBaud(MODEM_BAUDRATE);
  // GPS+GLONASS+GALILEO+SBAS+QZSS
  if (!modem.setGPSMode(3))
  {
    EBIKE_ERR("failed to set gps mode");
  }
  modem.configNMEASentence(1, 1, 1, 1, 1, 1);
  modem.setGPSOutputRate(1);
  modem.enableNMEA();
  if (!modem.enableAGPS())
  {
    EBIKE_ERR("enable GPS acceleration");
  }
}

void setup()
{
  Serial.begin(115200);

  // Turn on DC boost to power on the modem
#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

  // Set modem reset pin, reset modem
  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  // Turn on modem
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  // Set modem baud
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  wait_modem_start();

  while (wait_sim_online())
  {
    // TODO hang on certain failures
  }

#ifndef TINY_GSM_MODEM_SIM7672
  if (!modem.setNetworkMode(MODEM_NETWORK_AUTO))
  {
    EBIKE_ERR("Set network mode failed!");
  }
  EBIKE_DBG("Current network mode: ", modem.getNetworkModes());
#endif

#ifdef NETWORK_APN
  EBIKE_DBG("Set network apn: ", NETWORK_APN);
  modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
  while (!modem.waitResponse())
  {
    // TODO hang on certain failures
    EBIKE_ERR("Set network apn error!");
  }
#endif

  while (wait_network_registration())
  {
    // TODO hang
  }
  while (!modem.enableNetwork())
  {
    // TODO hang
    EBIKE_ERR("Enable network failed!");
  }
  delay(5000);
  EBIKE_NFO("Network IP: ", modem.getLocalIP());

  enable_gps();
}

static void displayInfo()
{
  // Show full information
  /*
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  */

  if (gps.location.isValid())
  {
    EBIKE_NFOF("Location: Lat=%.6f Lon=%.6f", gps.location.lat(), gps.location.lng());
    EBIKE_NFO("Satelites: ", gps.satellites.value());
  }
  else
  {
    EBIKE_NFO("Location: Unknown");
  }

  if (gps.date.isValid())
  {
    EBIKE_NFOF("Date: %02d/%02d/%04d", gps.date.month(), gps.date.day(), gps.date.year());
  }
  else
  {
    EBIKE_NFO("Date: Unknown");
  }

  if (gps.time.isValid())
  {
    EBIKE_NFOF("Time: %02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  }
  else
  {
    EBIKE_NFO("Time: Unknown");
  }
  EBIKE_NFO("--------------------------------");
}

/**
 * This custom version of delay() ensures that the gps object
 * is being "fed".
 */
static void smartDelay(unsigned long ms)
{
  int ch = 0;
  unsigned long start = millis();
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

void loop()
{
  displayInfo();

  if (millis() > check_interval)
  {
    if (modem.isEnableGPS() == false)
    {
      EBIKE_DBG("Restart GPS!");
      modem.enableGPS();
      delay(1000);
    }
    check_interval = millis() + 1000;
  }

  smartDelay(1000);

  if (millis() > 30000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS data received: check wiring"));
  }
}
