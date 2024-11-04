// Import the libraries
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// Connection To ESP
#define RXD2 17
#define TXD2 16
HardwareSerial mySerial(2); 

// Define the maximum number of peers
#define MAX_PEERS 4
#define SENSOR_FAN 1

// Structure to store the list of peer addresses
typedef struct {
  uint8_t peer_addr[6];
  int boardId;
  bool active;
  unsigned long mills;
} peer_t;
peer_t peers[MAX_PEERS];

// Structure for receiving sensor data
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

// Structure for sending fan control data
struct RelayData {
  int boardId;
  bool relay1;
  bool relay2;
  bool relay3;
  bool relay4;
};
RelayData myData;

// Print the received data in HEX
void printByteArray(const char* label, unsigned char* array, int length) {
  Serial.print(label);
  for (int i = 0; i < length; i++) {
    Serial.print(" ");
    Serial.print(array[i], HEX);
  }
  Serial.println();
}

// Add a new peer to the list
void addPeer(uint8_t* mac_addr, int boardId) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) {
      memcpy(peers[i].peer_addr, mac_addr, 6);
      peers[i].active = true;
      peers[i].boardId = boardId;
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, mac_addr, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
      } else {
        Serial.printf("Added new peer: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
      }
      break;
    }
  }
}

// Send fan speed and light status to receiver node
void sendData(RelayData &data) {
  Serial.printf("Data Sent To Board: %d\t Relay 1: %s\t Relay 2: %s\t Relay 3: %s\t Relay 4: %s \n", data.boardId, data.relay1, data.relay2, data.relay3, data.relay4);
  if (getPeer(data.boardId) != NULL) {
    esp_now_send(getPeer(data.boardId), (uint8_t*)&myData, sizeof(myData));
  }
}

// Clear the peer data
void clearPeer(const uint8_t* mac_addr) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (memcmp(peers[i].peer_addr, mac_addr, 6) == 0) {
      peers[i].active = false;
      removePeer(i);
      break;
    }
  }
}

// Remove a peer from the list
void removePeer(int index) {
  if (index >= 0 && index < MAX_PEERS) {
    esp_now_del_peer(peers[index].peer_addr);
    for (int i = index; i < MAX_PEERS - 1; i++) {
      peers[i] = peers[i + 1];
    }
    memset(&peers[MAX_PEERS - 1], 0, sizeof(peer_t));
  }
}

// Get the peer address by board ID
uint8_t* getPeer(int boardId) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].boardId == boardId) {
      return peers[i].peer_addr;
    }
  }
  return NULL;  // Return NULL if peer with boardId is not found
}

// Check if a peer is in the list
bool isPeerInList(uint8_t* mac_addr) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (memcmp(peers[i].peer_addr, mac_addr, 6) == 0) {
      return true;
    }
  }
  return false;
}

// Callback when data is recieved
void OnDataRecv(uint8_t* mac_addr, uint8_t* incomingData, uint8_t len) {
  SensorData data;
  memcpy(&data, incomingData, sizeof(SensorData));

  if (!isPeerInList(mac_addr)) {
    addPeer(mac_addr, data.boardId);
  }

  for (int i = 0; i < MAX_PEERS; i++) {
    if (memcmp(peers[i].peer_addr, mac_addr, 6) == 0) {
      peers[i].mills = millis();
      break;
    }
  }

  // Print data to Serial Monitor
  Serial.printf("Classroom ID: %d :- Temperature: %.2f, Humidity: %.2f, Air Quality: %.2f, Pressure: %.2f, Current: %.2f, Voltage: %.2f\n", data.boardId, data.temperature, data.humidity, data.gas, data.pressure, data.current, data.voltage);
  if (mySerial.available()) {
    mySerial.write((uint8_t*)&data, sizeof(data));
    delay(100); // Small delay between sending packets
    Serial.println("Data sent to receiver");
  } else {
    Serial.println("Data not sent");
  }
}

// callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Function to check if a peer is connected based on last activity
bool isPeerConnected(int boardId) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].boardId == boardId && peers[i].active) {
      // Consider connected if last communication was within 60 seconds
      if (millis() - peers[i].mills < 60000) {
        return true;
      }
    }
  }
  return false;
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

// Setup function
void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2); // Connect To ESP

  // Init WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  Serial.print("ESP32 Board MAC Address: ");
  readMacAddress();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Init the peer list
  for (int i = 0; i < MAX_PEERS; i++) {
    peers[i].active = false;
    peers[i].boardId = -1;
    peers[i].mills = 0;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));  // Register the callback function to receive the data

  esp_now_register_send_cb(OnDataSent);  // Register the callback function to send the data

  Serial.println("");
  Serial.println("Setup complete");
}

// Main loop
void loop() {
  // Periodically check if peers are connected
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && !isPeerConnected(peers[i].boardId)) {
      Serial.printf("Peer %d is disconnected\n", peers[i].boardId);
      peers[i].active = false;
    }
  }
}
