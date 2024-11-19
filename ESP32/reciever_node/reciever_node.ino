// 10:06:1c:f6:7a:34 + reciver Node 1

// Import the libraries for the ESP-NOW communication
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Library For Converting the data and senting it in the form of JSON
#include <ArduinoJson.h>

// Library for the DHT22 sensor : Temperature and Humidity
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Library for the BMP280 sensor : Temperature and Pressure
#include <Adafruit_BMP280.h>

// Library for the MQ135 sensor : Air/Gas Quality
#include <MQ135.h>

// Board ID [Just increase the number]
#define BOARD_ID 201

// Define: DHT11/22 Sensor Pins
#define DHTPIN 5
#define DHTTYPE DHT22
// #define DHTTYPE DHT11

// Define: BMP280 Sensor Pins
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define SEALEVELPRESSURE_HPA (1013.25)

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
Adafruit_BMP280 bmp;       // Define the BMP280 sensor
MQ135 mq135(MQ135PIN);     // Define the MQ135 sensor

// MAC Address of the receiver
uint8_t centralReceiver[] = { 0x88, 0x13, 0xbf, 0x63, 0xce, 0xb0 };
uint8_t NodeAddress[6];;

// JSON Data Structure
String recv_jsondata;
String send_jsondata;
StaticJsonDocument<256> doc_from_central;
StaticJsonDocument<256> doc_to_central;

// Get Mac Address
void readMacAddress() {
  // Get the MAC address of the ESP32
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, NodeAddress);
  
  if (ret == ESP_OK) {
    // Print the MAC address in the correct format
    Serial.printf("MAC Address: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                  NodeAddress[0], NodeAddress[1], NodeAddress[2], 
                  NodeAddress[3], NodeAddress[4], NodeAddress[5]);
  } else {
    // Print an error message if unable to fetch the MAC address
    Serial.println("Error: Failed to read MAC address.");
  }
}

// Function to read current from ACS712 sensor
float getCurrentAC() {
  int sensorValue = analogRead(ACS712PIN);
  float voltage = sensorValue * (3.3 / 4095.0);  // Adjusted for 3.3V and 12-bit ADC
  float current = (voltage - 2.5) / 0.066;       // Adjusted for sensor sensitivity
  return current;
}

// Callback when data is received
void OnDataRecv(uint8_t *mac, uint8_t *incomingData, uint8_t len) {
  char* buff = (char*)incomingData;
  recv_jsondata = String(buff);
  Serial.print("Received Data: ");
  Serial.println(recv_jsondata);
  
  DeserializationError error = deserializeJson(doc_from_central, recv_jsondata);
  if (!error) {
    bool relaypin1 = doc_from_central["relayPin1"];
    bool relaypin2 = doc_from_central["relayPin2"];
    bool relaypin3 = doc_from_central["relayPin3"];
    bool relaypin4 = doc_from_central["relayPin4"];
    setRelayState(RELAY_PIN1, relaypin1);
    setRelayState(RELAY_PIN2, relaypin2);
    setRelayState(RELAY_PIN3, relaypin3);
    setRelayState(RELAY_PIN4, relaypin4);
  } else {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
    return;
  }
  recv_jsondata = "";
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

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Display MAC Address
  readMacAddress();

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
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1)
      ;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL, Adafruit_BMP280::SAMPLING_X2, Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_X16, Adafruit_BMP280::STANDBY_MS_500);  // Operating Mode, Temp. oversampling, Pressure oversampling, Filtering, Standby time.

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

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

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);
}

unsigned long previousMillis = 0;

void loop() {
  unsigned long currentMillis = millis();

  // Wait for 10 seconds before sending the next reading
  if (currentMillis - previousMillis >= 10000) {
    previousMillis = currentMillis;
    // Read the sensor data
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
      Serial.println("Failed to read from DHT sensor!");
      return;
    }
    float airQuality = mq135.getCorrectedPPM(temperature, humidity);
    float pressure = bmp.readPressure() / 100000.0F;
    float current = getCurrentAC();
    float voltage = current * 230.0;

    char macAddress[18];
    snprintf(macAddress, sizeof(macAddress), "%02X:%02X:%02X:%02X:%02X:%02X", NodeAddress[0], NodeAddress[1], NodeAddress[2], NodeAddress[3], NodeAddress[4], NodeAddress[5]);

    // Prepare the JSON data
    doc_to_central["boardId"] = BOARD_ID;
    doc_to_central["temperature"] = temperature;
    doc_to_central["humidity"] = humidity;
    doc_to_central["airQuality"] = airQuality;
    doc_to_central["pressure"] = pressure;
    doc_to_central["current"] = current;
    doc_to_central["voltage"] = voltage;
    doc_to_central["macAddress"] = macAddress;
    serializeJson(doc_to_central, send_jsondata);
    
    // Send sensor data
    esp_now_send(centralReceiver, (uint8_t *) send_jsondata.c_str(), send_jsondata.length());
    send_jsondata = "";
  } 
}
