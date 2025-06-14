#include "ebike-log.hpp"

#include <memory>

#include <TinyGsmClient.h>
#include <SparkFunLSM6DS3.h>

#include "ebike-sms.hpp"
#include "ebike-battery.hpp"
#include "EBikeGPS.hpp"

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>

std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(debugger);
#else
std::shared_ptr<TinyGsm> modem = std::make_shared<TinyGsm>(SerialAT);
#endif // DUMP_AT_COMMANDS

// Global data
static EBikeGPS gps(modem);
static LSM6DS3 imu(I2C_MODE, 0x6B);
static bool traccar_enabled = TRACCAR_ENABLED;
static bool alarm_enabled = false;
static bool alarm_triggered = false;
// How long ago was the last move detection (in ms)
// Updated during light sleep
static uint32_t lastMovementAgo = 0;

static void restart()
{
  EBIKE_NFO("Restarting the system...");
  Serial.flush();
  delay(100);
  esp_restart();
}

static bool alert(const String &message)
{
  EBIKE_DBG("Sending SMS alert: ", message);
  if (!modem->sendSMS(MY_PHONE, message))
  {
    EBIKE_ERR("Failed to send SMS alert");
    return false;
  }
  return true;
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
    EBIKE_NFO(".");
    if (retry > 10)
    {
      RESET_MODEM_PWR;
      retry = 0;
    }
  }
  EBIKE_NFO("Modem started.");
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

static bool setup_accelerometer()
{
  if (imu.beginCore() != 0)
  {
    return false;
  }

  uint8_t errorAccumulator = 0;
  uint8_t dataToWrite = LSM6DS3_ACC_GYRO_BW_XL_200Hz |
                        LSM6DS3_ACC_GYRO_FS_XL_2g |
                        LSM6DS3_ACC_GYRO_ODR_XL_416Hz;

  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  // Set the ODR bit
  errorAccumulator += imu.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

  // Enable tap detection on X, Y, Z axis, but do not latch output
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x0E);

  // Set tap threshold
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_TAP_THS_6D, 0x03);

  // Set Duration, Quiet and Shock time windows
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_INT_DUR2, 0x7F);

  // Single & Double tap enabled (SINGLE_DOUBLE_TAP = 1)
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_WAKE_UP_THS, 0x80);

  // Single tap interrupt driven to INT1 pin -- enable latch
  errorAccumulator += imu.writeRegister(LSM6DS3_ACC_GYRO_MD1_CFG, 0x48);

  if (errorAccumulator)
  {
    return false;
  }

  // Configure interrupt pin
  gpio_set_direction(ACCEL_PIN, GPIO_MODE_INPUT);

  return true;
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
#endif

#ifdef NETWORK_APN
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
  if (!modem->setNetworkActive())
  {
    EBIKE_ERR("Enable network failed!");
    restart();
  }

  // Set SMS system into text mode
  modem->sendAT("+CMGF=1");
  modem->waitResponse();

  gps.enable(3);
  gps.bootstapWithGsm();

  // accelerometer and gyroscope initialization
  if (!setup_accelerometer())
  {
    EBIKE_ERR("Failed to setup accelerometer.");
  }

  alert("Hello, finished booting up.");
}

static bool processSmsCmds()
{
  SMS sms;
  if (readSms(modem, sms))
  {
    // delete latest SMS if we manage to read it
    deleteSMSByIndex(modem, 1);
  }
  if (!sms.valid)
  {
    // if SMS is not valid, skip it
    return false;
  }

  if (sms.sender != MY_PHONE)
  {
    EBIKE_NFO("SMS from unknown sender: ", sms.sender);
    return true;
  }

  // valid user interactions reset the sleep time
  lastMovementAgo = 0;

  sms.message.toUpperCase();
  if (sms.message == "RESTART")
  {
    alert("Restarting");
    restart();
  }
  else if (sms.message == "GPS")
  {
    alert("https://www.google.com/maps/place/" +
          String(gps.lat(), 8) + "," +
          String(gps.lon(), 8));
#ifdef EBIKE_DEBUG_BUILD
    gps.display();
#endif // EBIKE_GPS_DEBUG
  }
  else if (sms.message == "BAT")
  {
    double battery = readBattery();
    alert("Battery level: " + String(battery, 2) + "%");
  }
  else if (sms.message == "TRACK")
  {
    traccar_enabled = true;
    alert("Traccar enabled.");
  }
  else if (sms.message == "TRACKOFF")
  {
    traccar_enabled = false;
    alert("Traccar disabled.");
  }
  else if (sms.message == "ALARM")
  {
    alarm_enabled = true;
    alert("Alarm on.");
  }
  else if (sms.message == "ALARMOFF")
  {
    alarm_enabled = false;
    alert("Alarm off.");
  }
  else
  {
    EBIKE_ERR("Unknown SMS command: ", sms.message);
  }

  return true;
}

void light_sleep_delay(uint32_t ms)
{
  EBIKE_DBG("Entering light sleep for ", ms, " ms");

  esp_sleep_enable_ext0_wakeup(ACCEL_PIN, 1);
  esp_sleep_enable_timer_wakeup(ms * 1000);
  esp_light_sleep_start();
  // Disable wakeup sources after waking up
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);

  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0)
  {
    lastMovementAgo = 0;
    if (alarm_enabled)
    {
      alarm_enabled = false; // Disable alarm after triggering
      alarm_triggered = true;
    }
  }
  else
  {
    lastMovementAgo += ms;
  }
}

void modem_enter_sleep(uint32_t ms)
{
  EBIKE_DBG("Entering modem sleep for ", ms, " ms");

  // Pull up DTR to put the modem into sleep
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, HIGH);

  if (!modem->sleepEnable(true))
  {
    EBIKE_ERR("modem sleep failed!");
  }

  light_sleep_delay(ms);

  // Pull down DTR to wake up MODEM
  pinMode(MODEM_DTR_PIN, OUTPUT);
  digitalWrite(MODEM_DTR_PIN, LOW);

  // Wait modem wakeup
  light_sleep_delay(500);
}

void deep_sleep_until_movement()
{
  EBIKE_NFO("Entering deep sleep until movement is detected");
  Serial.flush();

  modem->poweroff();
#ifdef BOARD_POWERON_PIN
  // Turn on DC boost to power off the modem
  digitalWrite(BOARD_POWERON_PIN, LOW);
#endif
#ifdef MODEM_RESET_PIN
  // Keep it low during the sleep period. If the module uses GPIO5 as reset,
  // there will be a pulse when waking up from sleep that will cause the module to start directly.
  // https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/issues/85
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
  gpio_deep_sleep_hold_en();
#endif

  esp_sleep_enable_ext0_wakeup(ACCEL_PIN, 1);
  esp_deep_sleep_start();
}

void loop()
{
  bool is_power_on = modem->testAT(3000);
  if (!is_power_on)
  {
    restart();
  }

  if (lastMovementAgo >= DEEP_SLEEP_TIMEOUT)
  {
    alert("No movement detected for a while. Turning off...");
    deep_sleep_until_movement();
  }
  else if (lastMovementAgo >= ALARM_ENABLE_TIMEOUT && !alarm_enabled)
  {
    alert("Enabled alarm due to timeout.");
    alarm_enabled = true;
  }

  if (alarm_triggered)
  {
    // If the alarm is enabled and the accelerometer detects a tap, send an SMS
    EBIKE_NFO("Alarm triggered.");
    alert("Alarm triggered by accelerometer.");
  }
  alarm_triggered = false; // Reset the alarm trigger

  // Handle SMS commands
  while (processSmsCmds())
  {
    // Process SMS commands until no more valid SMS are found
  }

  bool gps_success = gps.update();
  bool post_success = true;
  if (traccar_enabled)
  {
    post_success = gps.post_location(TRACCAR_URL, TRACCAR_ID, readBattery());
  }

  if (gps_success)
  {
    if (post_success)
    {
      // If the positioning is successful, if posting the position is successful,
      // the ESP and modem are set to sleep (ultra low power consumption)
      modem_enter_sleep(REPORT_LOCATION_RATE_SECOND * 1000);
    }
    else
    {
      // If the positioning is successful, if posting the position fails,
      // set ESP to sleep and try again later (keep modem awake)
      light_sleep_delay(REPORT_LOCATION_RATE_SECOND * 1000);
    }
  }
  else
  {
    // If positioning is not successful, set ESP to sleep (keep modem awake)
    light_sleep_delay(15000);
  }
}
