# 🌊 FloodLogger

**LoRa-based water level sensor node for flood detection and early warning systems**  
Developed as part of a Bachelor's thesis using **LILYGO/TTGO ESP32** boards and **The Things Network (TTN)**.

---

## 📦 About the Project

FloodLogger is a low-power LoRaWAN sensor node that periodically measures the water level using an ultrasonic sensor and transmits the data via LoRa to TTN. It’s tailored for environmental monitoring in small rivers and creeks, especially in regions prone to flash floods.

> 🛠️ Optimized for:
> - **LILYGO T3 V1.6.1**
> - **TTGO LoRa32 SX1276 with OLED**

---

## 🔧 Requirements

Before you begin, make sure you have the following libraries installed in your **Arduino IDE**:

- 📡 LoRaWAN communication:  
  [`arduino-lmic`](https://github.com/mcci-catena/arduino-lmic)

- 🖥️ OLED display support *(optional)*:  
  [`esp8266-oled-ssd1306`](https://github.com/ThingPulse/esp8266-oled-ssd1306)

---

## 🚀 Getting Started

1. **Install the required libraries** (see above).

2. **Copy and configure the config file**:
   - Duplicate `config_example.h` and rename it to `config.h`
   - Fill in your TTN credentials (AppEUI, DevEUI, AppKey) and adjust the pin mappings if needed

3. **Select the correct board**:
   - In the Arduino IDE, go to **Tools > Board** and choose  
     `TTGO LoRa32-OLED V1`

4. **Set the upload speed**:
   - Go to **Tools > Upload Speed** and set it to `115200`

5. **Upload the code** to your board

6. **Power the device** and watch it transmit data to TTN!  
   If your board has an OLED, status updates will be displayed there.

---

## 📷 Optional OLED Display

If your board includes an OLED screen, the firmware supports displaying:

- Current water level
- Connection status
- Last transmission info

Make sure the correct pins are set in `config.h` and that the OLED library is installed.

---

## 📡 Data Transmission

FloodLogger uses the **LoRaWAN protocol** to send water level data at regular intervals.  
This data can be collected and visualized via TTN integrations such as:

- Node-RED
- InfluxDB & Grafana
- Custom dashboards or APIs

---

## 🤝 Acknowledgements

- 💬 Big thanks to the [DARC HamGroup LoRaWAN](https://www.p37.de/LoRaWAN/), Jürgen **DL8MA** for providing a working code example

---

## 📄 License

This project is released under the MIT License.  
Feel free to adapt and expand it for your own use!

---

> Made with ♥️ for a more resilient and connected future