#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#include "config.h"

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
    delay(2000);                                     // show message
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // 1min
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
  }
  else
  {
    long distance = readUltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    mydata[0] = (distance >> 8) & 0xFF;
    mydata[1] = distance & 0xFF;

    // show value and sending on display
    showDisplayMessage("Distance: " + String(distance) + " mm");
    LMIC_setTxData2(1, mydata, 2, 0);
    Serial.println(F("Packet queued"));
  }
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
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // 1 min
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

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  showDisplayMessage("Booting...");
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
    esp_sleep_enable_timer_wakeup(1 * 60 * 1000000); // 1 min
    esp_deep_sleep_start();
  }
}