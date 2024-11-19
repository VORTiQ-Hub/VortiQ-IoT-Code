// Import the libraries
#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

// Connection To ESP
#define RXD2 17
#define TXD2 16

// Define the maximum number of peers
#define MAX_PEERS 4
#define SENSOR_FAN 1

// Structure to store the list of peer addresses
typedef struct {
  uint8_t sent_addr[6];
  uint8_t peer_addr[6];
  int boardId;
} peer_t;
peer_t peers[MAX_PEERS];

// JSON Data Structure
StaticJsonDocument<256> doc_from_receiver;
StaticJsonDocument<256> doc_to_receiver;
String recv_jsondata;
String send_jsondata;

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
void addPeer(uint8_t* mac_addr, int boardId, bool isSender) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].boardId == (isSender ? boardId : -1)) {
      if (isSender) {
        memcpy(peers[i].sent_addr, mac_addr, 6);
      } else {
        memcpy(peers[i].peer_addr, mac_addr, 6);
      }

      peers[i].boardId = boardId;
      
      esp_now_peer_info_t peerInfo = {};
      memcpy(peerInfo.peer_addr, mac_addr, 6);
      peerInfo.channel = 0;
      peerInfo.encrypt = false;
      
      if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
      } else {
        if (isSender) {
          Serial.printf("Added new peer for sending: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        } else {
          Serial.printf("Added new peer on Recv: %02X:%02X:%02X:%02X:%02X:%02X\n", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        }
      }
      break;
    }
  }
}

// Get the peer address by board ID
uint8_t* getPeer(int boardId) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].boardId == boardId) {
      return peers[i].sent_addr;
    }
  }
  return NULL;  // Return NULL if peer with boardId is not found
}

// Check if a peer is in the list
bool isPeerInList(uint8_t* mac_addr) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (memcmp(peers[i].peer_addr, mac_addr, 6) == 0) {
      return true;
    } else if (memcmp(peers[i].sent_addr, mac_addr, 6) == 0) {
      return true;
    }
  }
  return false;
}

// Callback when data is recieved
void OnDataRecv(uint8_t* mac_addr, uint8_t* incomingData, uint8_t len) {
  char* buff = (char*)incomingData;
  recv_jsondata = String(buff);
  Serial.printf("Data Sent: ");Serial.println(recv_jsondata);
  
  DeserializationError error = deserializeJson(doc_from_receiver, recv_jsondata);
  if (!error) {
    int boardId = doc_from_receiver["boardId"];
    const char* macAddress = doc_from_receiver["macAddress"];

    uint8_t receiverAddress[6];
    sscanf(macAddress, "%02X:%02X:%02X:%02X:%02X:%02X", 
            &receiverAddress[0], &receiverAddress[1], &receiverAddress[2], 
            &receiverAddress[3], &receiverAddress[4], &receiverAddress[5]);

    if (!isPeerInList(mac_addr)) {
      addPeer(mac_addr, boardId, false);
      addPeer(receiverAddress, boardId, true);
    }

    serializeJson(doc_from_receiver, send_jsondata);
    Serial2.println(send_jsondata);
    send_jsondata = "";
  } else {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.c_str());
  }
  recv_jsondata ="";
  doc_from_receiver.clear();
}

// callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("\rLast Packet Send Status:  ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success\n" : "Delivery Fail\n");
}

// Setup function
void setup() {
  // Initialising UART Communication
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Connect To ESP

  // Initialising WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialising ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Initialising the peer list to empty
  for (int i = 0; i < MAX_PEERS; i++) {
    peers[i].boardId = -1;
  }

  // Defining Callback Functions
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  esp_now_register_send_cb(OnDataSent);

  Serial.println("");
  Serial.println("Setup complete");
}

// Main loop
void loop() {
  if (Serial2.available()) {
    recv_jsondata = Serial2.readStringUntil('\n');
    Serial.printf("Received Data: ");Serial.println(recv_jsondata);

    DeserializationError error = deserializeJson(doc_to_receiver, recv_jsondata);
    if (!error) {
      int boardID = doc_to_receiver["boardId"];
      serializeJson(doc_to_receiver, send_jsondata);     
      esp_err_t result = esp_now_send(getPeer(boardID), (uint8_t *) send_jsondata.c_str(), send_jsondata.length());
      send_jsondata = "";
    } else {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
    }
    recv_jsondata = "";
    doc_to_receiver.clear();
  }
}