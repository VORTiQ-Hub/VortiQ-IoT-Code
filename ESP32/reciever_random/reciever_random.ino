// Import the libraries for the ESP-NOW communication
#include <WiFi.h>
#include <esp_now.h>

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
  int boardId;
  int fanSpeed;
};

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

   // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

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
    return;
  }

  esp_now_register_send_cb(OnDataSent);  // Register the callback function to send the data
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
