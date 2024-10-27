// Import the libraries for the ESP-NOW communication
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Library for the DHT22 sensor : Temperature and Humidity
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Library for the BMP280 sensor : Temperature and Pressure
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>

// Library for the MQ135 sensor : Air/Gas Quality
#include <MQ135.h>

#include <Arduino.h>
#include <driver/ledc.h>

// Fan
#define PWM_PIN 18
#define PWM_CHANNEL 0
#define PWM_FREQUENCY 5000
#define PWM_RESOLUTION 2

#define BOARD_ID 3 // Board ID [Just increase the number]

// Define: DHT22 Sensor Pins
#define DHTPIN 5
#define DHTTYPE DHT22

// Define: BMP280 Sensor Pins
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

// Define: MQ135 Sensor Pins
#define MQ135PIN 36

// Define: ACS712 Sensor Pins
#define ACS712PIN 32

// Wi-Fi Credentials
#define WIFI_SSID "ESP_EF1089"
// #define WIFI_SSID "GNXS-2.4G-31C3F0"
// #define WIFI_SSID "kl.rab_3490"

DHT dht(DHTPIN, DHTTYPE);  // Define the DHT sensor type and pin
Adafruit_BMP280 bmp;       // Define the BMP280 sensor
MQ135 mq135(MQ135PIN);     // Define the MQ135 sensor

int32_t getWiFiChannel(const char *ssid) {
  if (int32_t n = WiFi.scanNetworks()) {
    for (uint8_t i=0; i<n; i++) {
      if (!strcmp(ssid, WiFi.SSID(i).c_str())) {
        return WiFi.channel(i);
      }
    }
  }
  return 0;
}

// MAC Address of the receiver
// uint8_t centralReceiver[] = { 0x10, 0x06, 0x1c, 0xF6, 0x8a, 0x08 };  // Home Wi-Fi
uint8_t centralReceiver[] = { 0xd8, 0xbc, 0x38, 0xe3, 0x04, 0xa4 };

// Structure to send data
struct SensorData {
  int boardId;
  int sensorType;
  float temperature;
  float humidity;
  float gas;
  float pressure;
  float current;
  float voltage;
};

SensorData myData;

// Structure to receive data
struct FanData {
  int boardId;
  int fanSpeed;
  bool lightStatus;
};

// Function to read current from ACS712 sensor
float getCurrentAC() {
  int sensorValue = analogRead(ACS712PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Adjusted for 3.3V and 12-bit ADC
  float current = (voltage - 2.5) / 0.066;       // Adjusted for sensor sensitivity
  return current;
}

// // Function to read data from sensor
// SensorData readSensorData() {
//   SensorData data;
//   data.boardId = BOARD_ID;
//   data.sensorType = 1;
//   // data.temperature = dht.readTemperature();
//   // data.humidity = dht.readHumidity();

//   // // Check if readings are valid
//   // if (isnan(data.temperature) || isnan(data.humidity)) {
//   //   Serial.println("Failed to read from DHT sensor!");
//   //   data.temperature = 0.0;
//   //   data.humidity = 0.0;
//   // }

//   // data.gas = mq135.getCorrectedPPM(data.temperature, data.humidity);
//   // data.pressure = bmp.readPressure() / 100000.0F;
//   // data.current = getCurrentAC();
//   data.temperature = random(0,100);
//   data.humidity = random(0,100);
//   data.gas = random(0,100);
//   data.pressure = 1;
//   data.current = random(0,100);
//   data.voltage = 245;  // Voltage is constant

//   return data;
// }

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (memcmp(incomingData, "ping", 4) == 0) {
    Serial.println("Ping received");
    esp_now_send(centralReceiver, (uint8_t *)"pong", 4);
  }

  FanData data;
  memcpy(&data, incomingData, sizeof(data));
  if (memcmp(mac, centralReceiver, 6) == 0 && data.boardId == BOARD_ID) {
    Serial.printf("Fan Speed: %d\n", data.fanSpeed);
    ledcWrite(PWM_CHANNEL, data.fanSpeed);
    Serial.printf("Light Status: %s\n", data.lightStatus);
  }
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Configure PWM channel and attach it to the GPIO
  ledcAttach(PWM_PIN, PWM_FREQUENCY, PWM_RESOLUTION);  // Setup PWM channel
  
  // Initialize the DHT sensor
  // dht.begin();

  // Initialize the BMP280 sensor
  // if (!bmp.begin(0x76)) {
  //   Serial.println("Could not find a valid BMP280 sensor, check wiring!");
  //   while (1)
  //     ;
  // }
  // bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);  // Operating Mode, Temp. oversampling, Pressure oversampling, Filtering, Standby time.

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));  // Register the callback function to receive the data

  // Register the peer
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, centralReceiver, 6);
  peerInfo.channel = getWiFiChannel(WIFI_SSID);  // Set your Wi-Fi channel
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  } else {
    Serial.println("Peer added successfully");
  }
  esp_now_register_send_cb(OnDataSent);  // Register the callback function to send the data
}

void loop() {
  // Read sensor data
  myData.boardId = BOARD_ID;
  myData.sensorType = 1;
  myData.temperature = random(0, 100);
  myData.humidity = random(0, 100);
  myData.gas = random(0, 100);
  myData.pressure = 1;
  myData.current = random(0, 100);
  myData.voltage = 245;  // Voltage is constant

  // Send sensor data
  esp_now_send(centralReceiver, (uint8_t *)&myData, sizeof(myData));

  // Wait for 2 seconds before sending the next reading
  delay(2000);
}
