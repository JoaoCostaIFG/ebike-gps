#include <Arduino.h>

#include "utilities.h"

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#include <TinyGsmClient.h>

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// It depends on the operator whether to set up an APN. If some operators do not
// set up an APN, they will be rejected when registering for the network. You
// need to ask the local operator for the specific APN.
#define NETWORK_APN "internet"

void setup() {
  Serial.begin(115200);
  // Turn on DC boost to power on the modem
#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

  // Set modem reset pin ,reset modem
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

  Serial.println("Start modem...");
  delay(3000);

  int retry = 0;
  while (!modem.testAT(1000)) {
    Serial.println(".");
    if (retry++ > 10) {
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
    }
  }
  Serial.println();
  delay(200);

  modem.sendAT("+SIMCOMATI");
  modem.waitResponse();

  // Check if SIM card is online
  SimStatus sim = SIM_ERROR;
  while (sim != SIM_READY) {
    sim = modem.getSimStatus();
    switch (sim) {
    case SIM_READY:
      Serial.println("SIM card online");
      break;
    case SIM_LOCKED:
      Serial.println(
          "The SIM card is locked. Please unlock the SIM card first.");
      // const char *SIMCARD_PIN_CODE = "123456";
      // modem.simUnlock(SIMCARD_PIN_CODE);
      break;
    default:
      break;
    }
    delay(1000);
  }

  if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
    Serial.println("Set network mode failed!");
  }

#ifdef NETWORK_APN
  Serial.printf("Set network apn : %s\n", NETWORK_APN);
  modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
  if (modem.waitResponse() != 1) {
    Serial.println("Set network apn error !");
  }
#endif

  // Check network registration status and network signal status
  int16_t sq;
  Serial.print("Wait for the modem to register with the network.");
  RegStatus status = REG_NO_RESULT;
  while (status == REG_NO_RESULT || status == REG_SEARCHING ||
         status == REG_UNREGISTERED) {
    status = modem.getRegistrationStatus();
    switch (status) {
    case REG_UNREGISTERED:
    case REG_SEARCHING:
      sq = modem.getSignalQuality();
      Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
      delay(1000);
      break;
    case REG_DENIED:
      Serial.println("Network registration was rejected, please check if the "
                     "APN is correct");
      return;
    case REG_OK_HOME:
      Serial.println("Online registration successful");
      break;
    case REG_OK_ROAMING:
      Serial.println(
          "Network registration successful, currently in roaming mode");
      break;
    default:
      Serial.printf("Registration Status:%d\n", status);
      delay(1000);
      break;
    }
  }
  Serial.println();

  delay(1000);

  String ueInfo;
  if (modem.getSystemInformation(ueInfo)) {
    Serial.print("Inquiring UE system information:");
    Serial.println(ueInfo);
  }

  if (!modem.enableNetwork()) {
    Serial.println("Enable network failed!");
  }

  delay(5000);

  String ipAddress = modem.getLocalIP();
  Serial.print("Network IP:");
  Serial.println(ipAddress);

  Serial.println("Enabling GPS/GNSS/GLONASS");
  while (!modem.enableGPS(MODEM_GPS_ENABLE_GPIO)) {
    Serial.print(".");
  }
  Serial.println();
  Serial.println("GPS Enabled");

  modem.setGPSBaud(MODEM_BAUDRATE);

  Serial.println("GPS acceleration is enabled");
  if (!modem.enableAGPS()) {
    Serial.println(" failed !!!");
  } else {
    Serial.println(" success!!!");
  }
}

void loop() {
  float lat2 = 0;
  float lon2 = 0;
  float speed2 = 0;
  float alt2 = 0;
  int vsat2 = 0;
  int usat2 = 0;
  float accuracy2 = 0;
  int year2 = 0;
  int month2 = 0;
  int day2 = 0;
  int hour2 = 0;
  int min2 = 0;
  int sec2 = 0;
  uint8_t fixMode = 0;
  for (;;) {
    Serial.println("Requesting current GPS/GNSS/GLONASS location");
    if (modem.getGPS(&fixMode, &lat2, &lon2, &speed2, &alt2, &vsat2, &usat2,
                     &accuracy2, &year2, &month2, &day2, &hour2, &min2,
                     &sec2)) {

      Serial.print("FixMode:");
      Serial.println(fixMode);
      Serial.print("Latitude:");
      Serial.print(lat2, 6);
      Serial.print("\tLongitude:");
      Serial.println(lon2, 6);
      Serial.print("Speed:");
      Serial.print(speed2);
      Serial.print("\tAltitude:");
      Serial.println(alt2);
      Serial.print("Visible Satellites:");
      Serial.print(vsat2);
      Serial.print("\tUsed Satellites:");
      Serial.println(usat2);
      Serial.print("Accuracy:");
      Serial.println(accuracy2);

      Serial.print("Year:");
      Serial.print(year2);
      Serial.print("\tMonth:");
      Serial.print(month2);
      Serial.print("\tDay:");
      Serial.println(day2);

      Serial.print("Hour:");
      Serial.print(hour2);
      Serial.print("\tMinute:");
      Serial.print(min2);
      Serial.print("\tSecond:");
      Serial.println(sec2);
      break;
    } else {
      Serial.println(
          "Couldn't get GPS/GNSS/GLONASS location, retrying in 15s.");
      delay(15000UL);
    }
  }
  Serial.println("Retrieving GPS/GNSS/GLONASS location again as a string");
  String gps_raw = modem.getGPSraw();
  Serial.print("GPS/GNSS Based Location String:");
  Serial.println(gps_raw);
  Serial.println("Disabling GPS");

  modem.disableGPS();

  while (1) {
    if (SerialAT.available()) {
      Serial.write(SerialAT.read());
    }
    if (Serial.available()) {
      SerialAT.write(Serial.read());
    }
    delay(1);
  }
}
