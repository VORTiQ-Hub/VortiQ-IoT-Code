// Import libraries for ESP-NOW communication
#include <ESP8266WiFi.h>
#include <espnow.h>

#define BOARD_ID 1  // Board ID [Just increase the number]

// MAC Address of the receiver
uint8_t centralReceiver[] = { 0xD8, 0x13, 0x2a, 0xef, 0x10, 0x88 };

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
  int id;
  int fanSpeed;
  bool lightStatus;
};

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
  myData.boardId = BOARD_ID;
  myData.sensorType = 1;
  myData.temperature = random(1,100);
  myData.humidity = random(1,100);
  myData.gas = random(1,100);
  myData.pressure = 1;
  myData.current = random(1,100);;
  myData.voltage = 245;  // Voltage is constant

  // Send sensor data
  esp_now_send(centralReceiver, (uint8_t *)&myData, sizeof(myData));

  // Wait for 2 seconds before sending the next reading
  delay(2000);
}