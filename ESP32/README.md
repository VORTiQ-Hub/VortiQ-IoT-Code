# VortiQ-IoT-Code: ESP32

## Overview
VortiQ-IoT is an IoT-based system utilizing **ESP32** microcontrollers for wireless communication and sensor data acquisition. The project supports multiple communication protocols, including **ESP-NOW** and **Wi-Fi**, to enable real-time monitoring and control of connected devices.

## Hardware Requirements
- **ESP32**: Microcontroller for Wi-Fi and IoT communication.
- **Sensors & Modules**:
  - **BMP180/BMP280**: Measures temperature and pressure.
  - **DHT11/DHT22**: Measures humidity and temperature.
  - **ACS712**: Measures current flow.
  - **MQ135**: Monitors air quality.
  - **RFID-RC522**: Enables RFID-based authentication and access control.

## Project Structure
1. **ESP Configuration**: Sets up ESP32 as a **Wi-Fi Access Point** or connects to an existing network.
2. **Data Communication**: Handles communication between multiple ESP modules using **ESP-NOW** or **Wi-Fi**.
3. **Data Processing**: Reads data from sensors and the RFID module, then structures it for real-time updates.
4. **User Interface**: Web-based dashboard to display real-time sensor data and control connected devices.

## Features
- **Multi-node Communication**: Enables ESP32 devices to communicate wirelessly.
- **Real-time Data Acquisition**: Gathers and processes sensor data instantly.
- **RFID Authentication**: Secure access control using RFID cards.
- **Low Power Consumption**: Optimized for efficient power usage in IoT applications.
