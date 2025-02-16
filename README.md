# VortiQ-IoT-Code: ESP32/ESP8266

## Overview
VortiQ-IoT is an IoT-based system utilizing **ESP32** or **ESP8266** microcontrollers for wireless communication and sensor data acquisition. The project supports multiple communication protocols, including **ESP-NOW** and **Wi-Fi**, to enable real-time monitoring and control of connected devices.

## Hardware Requirements
- **ESP32 or ESP8266**: Microcontrollers for Wi-Fi and IoT communication.
- **Sensors & Modules** (Optional, based on project requirements):
  - **BMP180/BMP280**: Measures temperature and pressure.
  - **DHT11/DHT22**: Measures humidity and temperature.
  - **ACS712**: Measures current flow.
  - **MQ135**: Monitors air quality.
  - **RFID-RC522**: Enables RFID-based authentication and access control.

## Project Structure
1. **ESP Configuration**: Sets up ESP32/ESP8266 as a **Wi-Fi Access Point** or connects to an existing network.
2. **Data Communication**: Handles communication between multiple ESP modules using **ESP-NOW** or **Wi-Fi**.
3. **Data Processing**: Reads data from sensors and the RFID module, then structures it for real-time updates.
4. **User Interface**: Web-based dashboard to display real-time sensor data and control connected devices.

## Features
- **Multi-node Communication**: Enables ESP32/ESP8266 devices to communicate wirelessly.
- **Real-time Data Acquisition**: Gathers and processes sensor data instantly.
- **RFID Authentication**: Secure access control using RFID cards.
- **Low Power Consumption**: Optimized for efficient power usage in IoT applications.

## Version Control
- **esp32**: Version 3.1.1 by espressif
  - Git: https://github.com/espressif/arduino-esp32
- **esp8266**: Version 3.1.2 by esp8266 community
  - Git: https://github.com/esp8266/Arduino

## Setup Instructions
1. Clone this repository:
   ```sh
   git clone https://github.com/your-repo/VortiQ-IoT-Code.git
   ```
2. Install dependencies using **Arduino IDE** or **PlatformIO**.
3. Upload the firmware to your ESP32/ESP8266.
4. Connect sensors and power up the module.
5. Monitor data via the **serial monitor** or web dashboard.

## License
This project is licensed under the **MIT License**.

## Contributors
- **Edwin C Shony**
- **Gopikrishna K M**
- **Rahul A B**
- **Sreerag Sreekanth**
