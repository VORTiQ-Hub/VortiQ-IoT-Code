#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ArduinoJson.h>

enum MessageType { PAIRING, DATA };
MessageType messageType;

// Connection To ESP-Firebase
#define RXD2 17
#define TXD2 16

// Define the maximum number of peers
#define MAX_PEERS 4
#define SENSOR_FAN 1

// JSON Data Structure
StaticJsonDocument<256> doc_from_firebase;
StaticJsonDocument<256> doc_to_firebase;
String recv_jsondata;
String send_jsondata;

// Structure to receive data
typedef struct struct_message {
  uint8_t msgType;
  int boardID;
  float temperature;
  float humidity;
  float gas;
  float pressure;
  float current;
  float voltage;
  int users;
} struct_message;

// Structure to send data
typedef struct struct_relay {
  uint8_t msgType;
  int boardID;
  int relay1;
  int relay2;
  int relay3;
  int relay4;
} struct_relay;

// Struct for pairing
typedef struct struct_pairing {
  uint8_t msgType;
  uint8_t id;
  int boardID;
  uint8_t macAddr[6];
} struct_pairing;

esp_now_peer_info_t slave;

struct_pairing pairingData;
struct_relay outgoingSetpoints;
struct_message incomingReadings;

// Stor MAC & BoardID
struct BoardInfo {
  uint32_t boardID;
  uint8_t mac[6];
} boardList[MAX_PEERS];

int boardCount = 0;

bool storeBoardID(uint32_t boardID, const uint8_t mac[6]) {
  // Check if the boardID already exists
  for (int i = 0; i < boardCount; i++) {
    if (boardList[i].boardID == boardID) {
      memcpy(boardList[i].mac, mac, 6);  // Update existing MAC
      Serial.println("Board ID updated.");
      return true;
    }
  }

  // If not found, add a new entry
  if (boardCount < MAX_PEERS) {
    boardList[boardCount].boardID = boardID;
    memcpy(boardList[boardCount].mac, mac, 6);
    boardCount++;  // Increase count
    Serial.println("New Board ID stored.");
    return true;
  } else {
    Serial.println("Board list is full!");
    return false;
  }
}

bool getMacFromBoardID(uint32_t boardID, uint8_t mac[6]) {
  for (int i = 0; i < boardCount; i++) {
    if (boardList[i].boardID == boardID) {
      memcpy(mac, boardList[i].mac, 6);
      return true;
    }
  }
  return false; // Not found
}

// Printing MAC Address
void printMAC(const uint8_t* mac_addr) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

// Add a new peer to the list
bool addPeer(const uint8_t* peer_addr) {
  memset(&slave, 0, sizeof(slave));
  const esp_now_peer_info_t* peer = &slave;
  memcpy(slave.peer_addr, peer_addr, 6);
  slave.channel = 0; // pick a channel
  slave.encrypt = false; // no encryption

  // check if the peer exists
  bool exists = esp_now_is_peer_exist(slave.peer_addr);
  if (exists) {
    // Slave already paired.
    Serial.println("Already Paired");
    return true;
  } else {
    esp_err_t addStatus = esp_now_add_peer(peer);
    if (addStatus == ESP_OK) {
      // Pair success
      Serial.println("Pair success");
      return true;
    } else {
      Serial.println("Pair failed");
      return false;
    }
  }
}

// callback when data is sent
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success to " : "Delivery Fail to ");
  printMAC(mac_addr);
  Serial.println();
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* incomingData, int len) {
  uint8_t type = incomingData[0];
  switch (type) {
    case DATA:
      send_jsondata = "";
      memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
      doc_to_firebase["dataType"] = "sensor";
      doc_to_firebase["boardID"] = incomingReadings.boardID;
      doc_to_firebase["temperature"] = incomingReadings.temperature;
      doc_to_firebase["humidity"] = incomingReadings.humidity;
      doc_to_firebase["gas"] = incomingReadings.gas;
      doc_to_firebase["pressure"] = incomingReadings.pressure;
      doc_to_firebase["current"] = incomingReadings.current;
      doc_to_firebase["voltage"] = incomingReadings.voltage;
      doc_to_firebase["users"] = incomingReadings.users;
      serializeJson(doc_to_firebase, send_jsondata);
      Serial.printf("Data Sent: ");
      Serial.println(send_jsondata);
      Serial2.println(send_jsondata);
      send_jsondata = "";
      doc_to_firebase.clear();
      break;

    case PAIRING:
      uint8_t clientMacAddress[6];
      memcpy(&pairingData, incomingData, sizeof(pairingData));
      Serial.println(pairingData.boardID);
      Serial.print("Pairing request from MAC Address: ");
      memcpy(clientMacAddress, pairingData.macAddr, sizeof(pairingData.macAddr));
      printMAC(clientMacAddress);
      Serial.println("");

      if (pairingData.id == 0) {
        if (pairingData.msgType == PAIRING) {
          bool status = addPeer(clientMacAddress);
          if (status) {
            storeBoardID(pairingData.boardID, clientMacAddress);
          }
          send_jsondata = "";
          doc_to_firebase["dataType"] = "MAC";
          doc_to_firebase["boardID"] = pairingData.boardID;
          for (int i = 0; i < 6; i++) {
            doc_to_firebase["MAC"][i] = pairingData.macAddr[i];
          }
          serializeJson(doc_to_firebase, send_jsondata);
          Serial.printf("Data Sent: ");
          Serial.println(send_jsondata);
          Serial2.println(send_jsondata);
        }
      }
      send_jsondata = "";
      doc_to_firebase.clear();
      break;
  }
}

void readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  // Init WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Printing MAC Address
  Serial.print("ESP32 Board MAC Address: ");
  readMacAddress();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Set Callback Routines
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  Serial.println("Setup Completed");
}

// Main loop
void loop() {
  if (Serial2.available()) {
    recv_jsondata = Serial2.readStringUntil('\n');
    Serial.printf("Received Data: ");
    Serial.println(recv_jsondata);

    DeserializationError error = deserializeJson(doc_from_firebase, recv_jsondata);
    if (!error) {
      int boardID = doc_from_firebase["boardID"];
      outgoingSetpoints.msgType = DATA;
      outgoingSetpoints.boardID = boardID;
      outgoingSetpoints.relay1 = doc_from_firebase["relayPin1"];
      outgoingSetpoints.relay2 = doc_from_firebase["relayPin2"];
      outgoingSetpoints.relay3 = doc_from_firebase["relayPin3"];
      outgoingSetpoints.relay4 = doc_from_firebase["relayPin4"];

      // Return Sent
      uint8_t MAC[6];
      getMacFromBoardID(boardID, MAC);
      esp_now_send(MAC, (uint8_t*)&outgoingSetpoints, sizeof(outgoingSetpoints));
      send_jsondata = "";
    } else {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.c_str());
    }
    recv_jsondata = "";
    doc_from_firebase.clear();
  }
}
