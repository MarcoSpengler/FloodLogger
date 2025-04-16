#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include "SSD1306Wire.h"

#include "config.h"


SSD1306Wire display(OLED_ADDRESS, OLED_SDA, OLED_SCL);

// Functions to get TTN keys from config.h
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

static uint8_t mydata[4];  // buffer for distance (2 bytes)
static osjob_t sendjob;

// Simple function to read distance from ultrasonic sensor
long readUltrasonicDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH, 25000); // timeout after 25ms
  long distance = duration * 0.1715; //conversion to mm
  return distance;
}

// OLED message function
void showDisplayMessage(String msg) {
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth() / 2, display.getHeight() / 2, msg);
  display.display();
}

// LMIC event handler
void onEvent(ev_t ev) {
  switch (ev) {
    case EV_JOINING:
      Serial.println(F("Joining..."));
      showDisplayMessage("Joining...");
      break;
    case EV_JOINED:
      Serial.println(F("Joined!"));
      showDisplayMessage("Joined network!");
      LMIC_setLinkCheckMode(0);
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("TX complete"));
      showDisplayMessage("Send complete!");

      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ACK"));
      }
      if (LMIC.dataLen > 0) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes"));
      }

      // replace with ESP.deepSleep(...) here later
      // ESP.deepSleep(4 * 60 * 60 * 1e6); // sleep for 4 hours
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("Join failed"));
      showDisplayMessage("Join failed!");
      break;
    default:
      Serial.print(F("Event: "));
      Serial.println(ev);
      break;
  }
}

// Prepare and send sensor data
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("Busy, not sending"));
    showDisplayMessage("Busy...");
  } else {
    long distance = readUltrasonicDistance();
    Serial.print("Distance: ");
    Serial.println(distance);

    // Encode distance as 2 bytes
    mydata[0] = (distance >> 8) & 0xFF;
    mydata[1] = distance & 0xFF;

    showDisplayMessage("Sending...");
    LMIC_setTxData2(1, mydata, 2, 0);
    Serial.println(F("Packet queued"));
  }
}

void setup() {
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  Serial.begin(115200);
  Serial.println(F("Booting"));

  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_16);
  showDisplayMessage("Booting...");

  os_init();
  LMIC_reset();
  do_send(&sendjob); // send first time
}

void loop() {
  os_runloop_once(); // nothing else needed here
}