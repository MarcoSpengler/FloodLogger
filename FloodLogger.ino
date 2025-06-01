#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#include "config.h"

#define BATTERY_CHECK_INTERVAL 50
#define SLEEP_DURATION_US (60ULL * 1000000ULL) // 60 seconds

SSD1306Wire display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

// Function declarations
void do_send(osjob_t *j);
void showDisplayMessage(String msg);
void initLoRa();
void checkSensors();
void waitForTXCompleteOrTimeout();

// LMIC keys from config
void os_getArtEui(u1_t *buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t *buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t *buf) { memcpy_P(buf, APPKEY, 16); }

static osjob_t sendjob;
static uint8_t mydata[4];

// Distance anomaly detection variables and logic
RTC_DATA_ATTR long lastDistances[3] = {0, 0, 0};
RTC_DATA_ATTR int distanceIndex = 0;
RTC_DATA_ATTR bool retryAfterAnomaly = false;
RTC_DATA_ATTR uint32_t wakeCount = 0;
#define BATTERY_CRITICAL_MV 3500

bool isAnomalous(long newDistance)
{
  if (newDistance == 0)
    return true;

  // Skip anomaly check if buffer not fully initialized
  for (int i = 0; i < 3; i++) {
    if (lastDistances[i] == 0) return false;
  }

  float mean = 0;
  for (int i = 0; i < 3; i++)
  {
    mean += lastDistances[i];
  }
  mean /= 3.0;

  float stddev = 0;
  for (int i = 0; i < 3; i++)
  {
    stddev += pow(lastDistances[i] - mean, 2);
  }
  stddev = sqrt(stddev / 3.0);

  float trend1 = lastDistances[1] - lastDistances[0];
  float trend2 = lastDistances[2] - lastDistances[1];
  float expectedTrend = (trend1 + trend2) / 2.0;
  float currentTrend = newDistance - lastDistances[2];

  return (abs(newDistance - mean) > 2 * stddev || abs(currentTrend - expectedTrend) > stddev);
}

void updateDistanceHistory(long newDistance)
{
  if (newDistance == 0) return; // skip invalid reads
  lastDistances[distanceIndex] = newDistance;
  distanceIndex = (distanceIndex + 1) % 3;
}

RTC_DATA_ATTR int dryCounter = 0;
bool shouldSend = false;
bool waitingForTXComplete = false;
bool sensorCheckedThisCycle = false;
bool isJoined = false;
bool lmicInitialized = false;
bool isJoining = false;

// Read distance from ultrasonic sensor
long readUltrasonicDistance()
{
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000); // 25ms timeout
  return duration * 0.1715;                                  // in mm
}

float readBatteryVoltage()
{
  int raw = analogRead(35);
  return (raw / 4095.0) * 3.3 * 2.0; // Adjust ratio if needed
}

// OLED message display
void showDisplayMessage(String msg)
{
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, msg);
  display.display();
}

// LoRaWAN events
void onEvent(ev_t ev)
{
  switch (ev)
  {
  case EV_JOINING:
    Serial.println(F("Joining..."));
    showDisplayMessage("Joining...");
    break;
  case EV_JOINED:
    Serial.println(F("Joined!"));
    showDisplayMessage("Joined network!");
    LMIC_setLinkCheckMode(0);
    isJoined = true;
    isJoining = false;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("TX complete"));
    showDisplayMessage("Send complete!");

    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ACK"));
    if (LMIC.dataLen > 0)
    {
      Serial.print(F("Received "));
      Serial.print(LMIC.dataLen);
      Serial.println(F(" bytes"));
    }

    delay(100); // let OLED settle

    waitingForTXComplete = false;

    // Sleep after send
    showDisplayMessage("Sleeping...");
    delay(2000); // show message
    wakeCount++;
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US); // 1min
    esp_deep_sleep_start();
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("Join failed"));
    showDisplayMessage("Join failed!");
    isJoining = false;
    break;
  default:
    Serial.print(F("Event: "));
    Serial.println(ev);
    break;
  }
}

// Prepare and send distance
void do_send(osjob_t *j)
{
  if (!(shouldSend) || waitingForTXComplete)
    return;

  if (LMIC.opmode & OP_TXRXPEND)
  {
    Serial.println(F("Busy, not sending"));
    showDisplayMessage("Busy...");
    return;
  }

  long distance = readUltrasonicDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  // Handle retry after anomaly
  if (retryAfterAnomaly)
  {
    retryAfterAnomaly = false;

    if (isAnomalous(distance))
    {
      // Confirmed anomaly: send fault code -1
      Serial.println("Confirmed anomaly - sending fault code");
      showDisplayMessage("Sending fault (-1)");
      mydata[0] = 0xFF;
      mydata[1] = 0xFF;
      LMIC_setTxData2(1, mydata, 2, 0);
      Serial.println(F("Fault packet queued"));
      return;
    }
    // Valid measurement after retry: continue to send normally
  }
  else
  {
    // First anomaly detected: schedule retry
    if (isAnomalous(distance))
    {
      Serial.println("Anomaly detected - sleeping before retry");
      showDisplayMessage("Anomaly - retrying");
      retryAfterAnomaly = true;
      wakeCount++;
      esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US);
      esp_deep_sleep_start();
      return;
    }
    // Valid first measurement: record it
    updateDistanceHistory(distance);
  }

  if (wakeCount >= BATTERY_CHECK_INTERVAL)
  {
    float voltage = readBatteryVoltage();
    uint16_t batt_mV = voltage * 1000;
    if (batt_mV < BATTERY_CRITICAL_MV)
    {
      int16_t negBattery = -((int16_t)batt_mV);
      Serial.print("Battery low, sending: ");
      Serial.println(negBattery);
      mydata[0] = (negBattery >> 8) & 0xFF;
      mydata[1] = negBattery & 0xFF;
      showDisplayMessage("Low bat: " + String(voltage, 2) + "V");
      LMIC_setTxData2(1, mydata, 2, 0);
      wakeCount = 0;
      return;
    }
    wakeCount = 0;
  }

  // Encode and send normal measurement
  mydata[0] = (distance >> 8) & 0xFF;
  mydata[1] = distance & 0xFF;
  showDisplayMessage("Distance: " + String(distance) + " mm");
  LMIC_setTxData2(1, mydata, 2, 0);
  Serial.println(F("Packet queued"));
}

void initLoRa()
{
  if (!lmicInitialized)
  {
    os_init();
    LMIC_reset();
    lmicInitialized = true;
    isJoined = false;
    isJoining = false;
  }
  if (!isJoined && !isJoining)
  {
    LMIC_startJoining();
    isJoining = true;
  }
}

void checkSensors()
{
  digitalWrite(SENSE_OUT, HIGH);
  delay(40);
  int val = analogRead(SENSE_IN);
  digitalWrite(SENSE_OUT, LOW);

  Serial.print("Analog value: ");
  Serial.println(val);

  if (true)
  {
    shouldSend = true;
    dryCounter = 0;
    Serial.println("Wet - sending");
    // showDisplayMessage
  }
  else
  {
    dryCounter++;
    Serial.print("Dry (");
    Serial.print(dryCounter);
    Serial.println(")");
    shouldSend = (dryCounter >= 3);
    if (shouldSend)
      dryCounter = 0;
  }

  sensorCheckedThisCycle = true;
}

void waitForTXCompleteOrTimeout()
{
  unsigned long txStart = millis();
  while (waitingForTXComplete && millis() - txStart < 8000)
  {
    os_runloop_once(); // let LMIC do its thing
  }

  if (waitingForTXComplete)
  {
    Serial.println("TX timeout - forcing sleep");
    showDisplayMessage("TX timeout...");
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    wakeCount++;
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US); // 1 min
    esp_deep_sleep_start();
  }
}

void setup()
{
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  pinMode(SENSE_OUT, OUTPUT);
  pinMode(SENSE_IN, INPUT_PULLDOWN);

  Serial.begin(115200);
  Serial.println(F("Booting"));

  analogReadResolution(12);
  analogSetPinAttenuation(35, ADC_11db);

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  showDisplayMessage("Booting...");

  // // DEBUG, show ultrasonic distance
  // Serial.println("Testing ultrasonic sensor...");
  // long testDistance = readUltrasonicDistance();
  // Serial.print("Test distance: ");
  // Serial.println(testDistance);
  // if (testDistance > 0)
  // {
  //   showDisplayMessage("Ultrasonic OK");
  // }
  // else
  // {
  //   showDisplayMessage("Ultrasonic error!");
  //   Serial.println("Ultrasonic sensor error!");
  // }
}

void loop()
{
  // Only check sensors once per wake-up cycle
  if (!sensorCheckedThisCycle)
  {
    checkSensors();
  }

  if (shouldSend)
  {
    initLoRa();

    if (!isJoined)
    {
      os_runloop_once(); // continue LMIC stack to join network
      return;
    }

    if (!waitingForTXComplete)
    {
      do_send(&sendjob);
      waitingForTXComplete = true;
    }

    waitForTXCompleteOrTimeout();
  }
  else
  {
    showDisplayMessage("Dry - sleep...");
    delay(2000);
    digitalWrite(LED_BUILTIN, LOW);
    wakeCount++;
    esp_sleep_enable_timer_wakeup(SLEEP_DURATION_US); // 1 min
    esp_deep_sleep_start();
  }
}