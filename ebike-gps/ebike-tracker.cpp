#include "ebike-conf.h"

#include <Arduino.h>
#include <memory>

#include "modem_utilities.h"
#include <TinyGsmClient.h>

#include "ebike-sms.hpp"
#include "ebike-log.hpp"
#include "ebike-battery.hpp"
#include "EBikeGPS.hpp"

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(debugger);
#else
std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(SerialAT);
#endif // DUMP_AT_COMMANDS

// Global data
static EBikeGPS gps(modem);
static size_t err_count = 0;

static void restart()
{
  EBIKE_NFO("Power Off , restart device");
  Serial.flush();
  delay(100);
  esp_restart();
}

static void start_modem()
{
#define RESET_MODEM_PWR                   \
  {                                       \
    digitalWrite(BOARD_PWRKEY_PIN, LOW);  \
    delay(100);                           \
    digitalWrite(BOARD_PWRKEY_PIN, HIGH); \
    delay(1000);                          \
    digitalWrite(BOARD_PWRKEY_PIN, LOW);  \
  }

  // Turn on modem
  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  RESET_MODEM_PWR;

  // Set modem baud
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

  EBIKE_DBG("Starting modem->..");
  delay(3000);
  for (unsigned retry = 0; !modem->testAT(1000); ++retry)
  {
    EBIKE_DBG(".");
    if (retry > 10)
    {
      RESET_MODEM_PWR;
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
    sim = modem->getSimStatus();
    switch (sim)
    {
    case SIM_READY:
      EBIKE_DBG("SIM online.");
      break;
    case SIM_LOCKED:
#ifdef SIMCARD_PIN
      if (!modem->simUnlock(SIMCARD_PIN))
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

    status = modem->getRegistrationStatus();
    switch (status)
    {
    case REG_UNREGISTERED:
    case REG_SEARCHING:
      EBIKE_DBG("Searching signal quality: ", modem->getSignalQuality());
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
  if (modem->getSystemInformation(ueInfo))
  {
    EBIKE_DBG("Inquiring UE system information: ", ueInfo);
  }

  return 0;
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

  // Set ring pin input
  pinMode(MODEM_RING_PIN, INPUT_PULLUP);

  // adc setting start
  // You don't need to set it, because the values ​​are all default. The current version is Arduino 3.0.4, and the subsequent versions are uncertain.
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);
#if CONFIG_IDF_TARGET_ESP32
  analogSetWidth(12);
#endif
  // adc setting end

  start_modem();

  if (wait_sim_online() != 0)
  {
    restart();
  }

#ifndef TINY_GSM_MODEM_SIM7672
  if (!modem->setNetworkMode(MODEM_NETWORK_AUTO))
  {
    EBIKE_ERR("Set network mode failed!");
  }
  EBIKE_DBG("Current network mode: ", modem->getNetworkModes());
#endif

#ifdef NETWORK_APN
  EBIKE_DBG("Set network apn: ", NETWORK_APN);
  modem->sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
  if (!modem->waitResponse())
  {
    EBIKE_ERR("Set network apn error!");
    restart();
  }
#endif

  if (wait_network_registration() != 0)
  {
    restart();
  }
  if (!modem->enableNetwork())
  {
    EBIKE_ERR("Enable network failed!");
    restart();
  }

  // Set SMS system into text mode
  modem->sendAT("+CMGF=1");
  modem->waitResponse();

  modem->https_begin();
  delay(5000);
  EBIKE_NFO("Network IP: ", modem->getLocalIP());
  modem->https_end();

  gps.enable(3, GPS_1HZ);
  gps.bootstapWithGsm();
}

static bool processSmsCmds()
{
  SMS sms = readSms(modem);
  if (!sms.valid)
  {
    return false;
  }
  deleteSMSByIndex(modem, 1);

  if (sms.sender != MY_PHONE)
  {
    EBIKE_NFO("SMS from unknown sender: ", sms.sender);
    return false;
  }

  EBIKE_NFO("Processing SMS command: ", sms.message);

  sms.message.toUpperCase();
  if (sms.message == "RESTART")
  {
    restart();
  }
  else if (sms.message == "GPS")
  {
    modem->sendSMS(MY_PHONE, "GPS location: " + String(gps.lat(), 8) + "," +
                                      String(gps.lon(), 8));
    gps.display();
  }

  return true;
}

void loop()
{
  // Check if the modem is responsive, otherwise reboot
  bool isPowerOn = modem->testAT(3000);
  if (!isPowerOn)
  {
    restart();
  }

#ifdef EBIKE_DEBUG_BUILD
  gps.display();
#endif // EBIKE_DEBUG_BUILD

  processSmsCmds();

  if (!gps.post_location(TRACCAR_URL, TRACCAR_ID, readBattery()))
  {
    err_count++;
  }
  else
  {
    err_count = 0;
  }
  if (err_count > 10)
  {
    EBIKE_ERR("Too many errors, restarting...");
    restart();
  }

  gps.delay(3000UL);
}
