# VortiQ-IoT-Code

## Hardware Requirements

- <u>**ESP32 or ESP8266**</u>: Microcontrollers for Wi-Fi and IoT communication.
- <u>**Sensors**</u>: (Optional, based on project requirements)
  - BMP180/BMP280: For temperature and pressure data.
  - DHT11/DHT22: For humidity and temperature data.
  - ACS712: For current measurement.
  - MQ135: For air quality measurement.

## Project Structure

1. **ESP Configuration**: Sets up ESP32/ESP8266 as a Wi-Fi Access Point or connects to an existing network.
2. **Data Communication**: Communication between multiple ESP modules using ESP-NOW or Wi-Fi.
3. **Data Processing**: Reads data from sensors and structures it for real-time updates.