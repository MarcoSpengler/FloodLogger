// config.h

#ifndef CONFIG_H
#define CONFIG_H

// ------------------------
// TTN LoRaWAN Credentials
// ------------------------
static const u1_t PROGMEM APPEUI[8]  = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED, 0xBA, 0xBE }; // LSB
static const u1_t PROGMEM DEVEUI[8]  = { 0xBA, 0xAD, 0xF0, 0x0D, 0xCA, 0xFE, 0xBE, 0xEF }; // LSB
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF }; // MSB

// ------------------------
// Pin Definitions
// ------------------------

// Ultrasonic sensor
#define ULTRASONIC_TRIG_PIN 12
#define ULTRASONIC_ECHO_PIN 34

// OLED I2C display
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_ADDRESS 0x3C

// LoRa module (SX1276)
#define LORA_SCK  5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS   18
#define LORA_RST  23
#define LORA_DIO0 26
#define LORA_DIO1 33
#define LORA_DIO2 32

// ------------------------
// General Configuration
// ------------------------

#define LED_BUILTIN 25

#define TX_INTERVAL 60 // in seconds

// LMIC pin mapping
const lmic_pinmap lmic_pins = {
  .nss = LORA_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = LORA_RST,
  .dio = {LORA_DIO0, LORA_DIO1, LORA_DIO2},
};

#endif