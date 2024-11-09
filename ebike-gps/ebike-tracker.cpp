#include "ebike-conf.h"

#include <Arduino.h>
#include <memory>

#include "modem_utilities.h"
#include <TinyGsmClient.h>

#include "ebike-log.hpp"
#include "EBikeGPS.hpp"

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(debugger);
#else
std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(SerialAT);
#endif

// Global data
static EBikeGPS gps(modem);

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

static void gsm_location()
{
  // You need to wait for the network to be activated before you can use the LBS function.
  // By default, the network is automatically activated after the modem is started.
  float lat = 0;
  float lon = 0;
  float accuracy = 0;
  int year = 0;
  int month = 0;
  int day = 0;
  int hour = 0;
  int min = 0;
  int sec = 0;

  while (!modem->getGsmLocation(&lat, &lon, &accuracy, &year, &month, &day, &hour,
                                &min, &sec))
  {
    EBIKE_DBG("Couldn't get GSM location, retrying...");
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

  // Set ring pin input
  pinMode(MODEM_RING_PIN, INPUT_PULLUP);

  start_modem();

  while (wait_sim_online())
  {
    // TODO hang on certain failures
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
  while (!modem->waitResponse())
  {
    // TODO hang on certain failures
    EBIKE_ERR("Set network apn error!");
  }
#endif

  while (wait_network_registration())
  {
    // TODO hang
  }
  while (!modem->enableNetwork())
  {
    // TODO hang
    EBIKE_ERR("Enable network failed!");
  }
  delay(5000);
  EBIKE_NFO("Network IP: ", modem->getLocalIP());

  gps.enable(3, GPS_5HZ);
}

void loop()
{
  // String gsm_location = modem->getGsmLocation();
  // EBIKE_DBG(gsm_location);

  gps.display();
  gps.delay(1000UL);
}
