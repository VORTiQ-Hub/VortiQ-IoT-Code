// Import the libraries for the ESP-NOW communication
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// Library for the DHT22 sensor : Temperature and Humidity
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Library for the BMP180 sensor : Temperature and Pressure
#include <Adafruit_BMP085.h>

// Library for the MQ135 sensor : Air/Gas Quality
#include <MQ135.h>

// Board ID [Just increase the number]
#define BOARD_ID 201

// Define: DHT11/22 Sensor Pins
#define DHTPIN 5
#define DHTTYPE DHT11
// #define DHTTYPE DHT22

// Define: MQ135 Sensor Pins
#define MQ135PIN 35

// Define: ACS712 Sensor Pins
#define ACS712PIN 32

// Control 4 Relay
#define RELAY_PIN1 27
#define RELAY_PIN2 26
#define RELAY_PIN3 25
#define RELAY_PIN4 33

DHT dht(DHTPIN, DHTTYPE);  // Define the DHT sensor type and pin
Adafruit_BMP085 bmp;       // Define the BMP280 sensor
MQ135 mq135(MQ135PIN);     // Define the MQ135 sensor

// MAC Address of the receiver
uint8_t centralReceiver[] = { 0x88, 0x13, 0xbf, 0x63, 0xce, 0xb0 };

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
struct RelayData {
  int boardId;
  bool relay1;
  bool relay2;
  bool relay3;
  bool relay4;
};

// Function to read current from ACS712 sensor
float getCurrentAC() {
  int sensorValue = analogRead(ACS712PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Adjusted for 3.3V and 12-bit ADC
  float current = (voltage - 2.5) / 0.066;       // Adjusted for sensor sensitivity
  return current;
}

// Function to read data from sensor
SensorData readSensorData() {
  SensorData data;
  data.boardId = BOARD_ID;
  data.sensorType = 1;
  data.temperature = dht.readTemperature();
  data.humidity = dht.readHumidity();

  // Check if readings are valid
  if (isnan(data.temperature) || isnan(data.humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    data.temperature = 0.0;
    data.humidity = 0.0;
  }

  data.gas = mq135.getCorrectedPPM(data.temperature, data.humidity);
  data.pressure = bmp.readPressure() / 100000.0F;
  data.current = getCurrentAC();
  data.temperature = random(0, 100);
  data.humidity = random(0, 100);
  data.gas = random(0, 100);
  data.pressure = 1;
  data.current = random(0, 100);
  data.voltage = 245;  // Voltage is constant

  return data;
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  if (memcmp(incomingData, "ping", 4) == 0) {
    Serial.println("Ping received");
    esp_now_send(centralReceiver, (uint8_t *)"pong", 4);
  }

  // Ensure the incoming data length is correct
  if (len != sizeof(RelayData)) {
    Serial.println("Received data length mismatch");
    return;
  }

  RelayData data;
  memcpy(&data, incomingData, sizeof(data));
  // Check if the data is from the central receiver and intended for this board
  if (memcmp(mac, centralReceiver, 6) == 0 && data.boardId == BOARD_ID) {
    setRelayState(RELAY_PIN1, data.relay1);
    setRelayState(RELAY_PIN2, data.relay2);
    setRelayState(RELAY_PIN3, data.relay3);
    setRelayState(RELAY_PIN4, data.relay4);
  }
}

// Helper function to set relay state
void setRelayState(uint8_t relayPin, bool state) {
  digitalWrite(relayPin, state ? HIGH : LOW);
}

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Initialize serial monitor
  Serial.begin(115200);

  // Initialize relay control pins
  pinMode(RELAY_PIN1, OUTPUT);
  pinMode(RELAY_PIN2, OUTPUT);
  pinMode(RELAY_PIN3, OUTPUT);
  pinMode(RELAY_PIN4, OUTPUT);

  // Turn off all relays at startup
  digitalWrite(RELAY_PIN1, LOW);
  digitalWrite(RELAY_PIN2, LOW);
  digitalWrite(RELAY_PIN3, LOW);
  digitalWrite(RELAY_PIN4, LOW);

  // Initialize the DHT sensor
  dht.begin();

  // Initialize the BMP280 sensor
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1)
      ;
  }

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
  peerInfo.channel = 0;  // Set your Wi-Fi channel
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
  myData = readSensorData();

  // Send sensor data
  esp_now_send(centralReceiver, (uint8_t *)&myData, sizeof(myData));

  // Wait for 2 seconds before sending the next reading
  delay(2000);
}
