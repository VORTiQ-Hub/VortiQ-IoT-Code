// Import libraries for ESP-NOW communication
#include <ESP8266WiFi.h>
#include <espnow.h>

// Library for the DHT22 sensor
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Library for the BMP280 sensor
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

// Library for the MQ135 sensor
#include <MQ135.h>

#define BOARD_ID 1  // Board ID [Just increase the number]

// Define pins
#define DHTPIN 5                        // DHT11 sensor pin
#define DHTTYPE DHT11                   // DHT11 sensor type
#define BMP_SCK 14                      // BMP280 sensor pins
#define BMP_MISO 12                     // BMP280 sensor pins
#define BMP_MOSI 13                     // BMP280 sensor pins
#define BMP_CS 15                       // BMP280 sensor pins
#define SEALEVELPRESSURE_HPA (1013.25)  // Default sea level pressure
#define MQ135PIN A0                     // MQ135 sensor pin
#define ACS712PIN A1                    // ACS712 sensor pin

DHT dht(DHTPIN, DHTTYPE);  // Define the DHT sensor type and pin
Adafruit_BMP280 bmp;       // Define the BMP280 sensor
MQ135 mq135(MQ135PIN);     // Define the MQ135 sensor

// MAC Address of the receiver
uint8_t centralReceiver[] = { 0x40, 0x91, 0x51, 0x45, 0x2F, 0x66 };

// Structure to send data
struct SensorData {
  int id;
  int sensorType;
  float temperature;
  float humidity;
  float gas;
  float pressure;
  float current;
};

SensorData myData;

// Structure to receive data
struct FanData {
  int id;
  int fanSpeed;
  bool lightStatus;
};

// Function to read current from ACS712 sensor
int getCurrentAC() {
  int sensorValue = analogRead(A0);
  float voltage = sensorValue * (5.0 / 1023.0);
  float current = (voltage - 2.5) / 0.066;
  return current;
}

// Function to read data from sensor
SensorData readSensorData() {
  SensorData data;
  data.id = BOARD_ID;
  data.sensorType = 1;
  // data.temperature = dht.readTemperature();
  // data.humidity = dht.readHumidity();
  // data.gas = mq135.getCorrectedPPM(data.temperature, data.humidity);
  // data.pressure = bmp.readPressure();
  // data.current = getCurrentAC();
  data.temperature = random(0,100);
  data.humidity = random(0,100);
  data.gas = random(0,100);
  data.pressure = 1;
  data.current = random(0,100);
  data.voltage = 245;
  return data;
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (strcmp((char *)incomingData, "ping") == 0) {
    Serial.println("Ping received");
    const char* message = "pong";
    esp_now_send(centralReceiver, (uint8_t *)message, strlen(message));
  } else {
    FanData data;
    memcpy(&data, incomingData, sizeof(data));
    if (mac == centralReceiver && data.id == BOARD_ID) {
      Serial.print("Fan Speed: ");
      Serial.println(data.fanSpeed);
      Serial.print("Light Status: ");
      Serial.println(data.lightStatus);
    }
  }
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // // Initialize the DHT sensor
  // dht.begin();

  // // Initialize the BMP280 sensor
  // if (!bmp.begin()) {
  //   Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  //   while (1)
  //     ;
  // }

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Initialize ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set ESP-NOW role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Register callbacks for sending data
  esp_now_register_send_cb(OnDataSent);

  esp_now_add_peer(centralReceiver, ESP_NOW_ROLE_COMBO, 1, NULL, 0);

  // Register callback for receiving data
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  // Read sensor data
  myData = readSensorData();

  // Send sensor data
  esp_now_send(centralReceiver, (uint8_t *)&myData, sizeof(myData));

  // Wait for 2 seconds before sending the next reading
  delay(2000);
}