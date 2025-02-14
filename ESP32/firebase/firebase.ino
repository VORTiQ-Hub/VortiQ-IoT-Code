#include <WiFi.h>
#include <ArduinoJson.h>
#include <FirebaseESP32.h>
#include <HardwareSerial.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// Connection To Central ESP
#define RXD2 16
#define TXD2 17

// Network credentials
const char* ssid1 = "GNXS-2.4G-31C3F0";
const char* password1 = "25051971";
const char* ssid2 = "kl.rab_3490";
const char* password2 = "KL.rab_3490";

// Firebase project API key and RTDB URL
#define API_KEY "AIzaSyAhVQx6I1juQkA_oVnlvZmoq1mJ2XooM6Q"
#define DATABASE_URL "https://vortiq-cee69-default-rtdb.firebaseio.com"
#define DATABASE_SECRET "Fkh488R9xwVg77Qy9qfKbUtjRYCCgkyOUlFwgK24"

// Firebase user credentials
#define USER_EMAIL "device@gmail.com"
#define USER_PASSWORD "device@123"

FirebaseData firebaseData;
FirebaseAuth firebaseAuth;
FirebaseConfig firebaseConfig;

// JSON Data Structure
String recv_jsondata;
String send_jsondata;
StaticJsonDocument<256> doc_to_firebase;
StaticJsonDocument<256> doc_from_firebase;

// Function to connect to Wi-Fi
void connectWiFi(const char* ssid, const char* password) {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Wait until connected
  int retryCount = 0;
  while (WiFi.status() != WL_CONNECTED && retryCount < 20) {
    delay(500);
    Serial.print(".");
    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Wi-Fi!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect.");
  }
}

void sentToFirebaseMAC(int boardId, String mac_address) {
  String path = "/devices/" + String(boardId) + "/data";
  FirebaseJson json;
  json.set("Room ID", boardId);
  json.set("MAC Address", mac_address);

  FirebaseJsonData jsonData; // Object to hold the retrieved data

  // Fetch existing data from Firebase
  if (Firebase.getJSON(firebaseData, path)) {
    FirebaseJson existingJson = firebaseData.jsonObject();

    String existingMac;
    int existingBoardId;

    // Extract existing values using FirebaseJsonData
    if (existingJson.get(jsonData, "MAC Address")) {
      existingMac = jsonData.stringValue;
    }

    if (existingJson.get(jsonData, "Room ID")) {
      existingBoardId = jsonData.intValue;
    }

    // Compare with new values
    if (existingBoardId == boardId && existingMac == mac_address) {
      Serial.println("No changes detected. Data remains the same.");
      return; // Exit without updating Firebase
    }
  }

  // If data is different or doesn't exist, update Firebase
  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("Data Section Data Sent Successfully.");
  } else {
    Serial.print("Failed to add data section: ");
    Serial.println(firebaseData.errorReason());
  }
}

void sentToFirebase(int boardID, float temperature, float humidity, float gas, float pressure, float current, float voltage) {
  if (WiFi.status() != WL_CONNECTED) {  // Check WiFi connection
    Serial.println("WiFi not connected. Unable to send data.");
    return;
  }

  String path = "/devices/" + String(boardID) + "/sensor";
  FirebaseJson json;
  json.set("temperature", temperature);
  json.set("humidity", humidity);
  json.set("airQuality", gas);
  json.set("pressure", pressure);
  json.set("current", current);
  json.set("voltage", voltage);

  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("Sensor Section Data Sent Successfully.");
  } else {
    Serial.print("Failed to send data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void getFromFirebase(int boardID) {
  doc_from_firebase["boardID"] = boardID;

  for(int i=1;i<=4;i++) {
    String path = "/devices/" + String(boardID) + "/relay/" + String(i);

    if (Firebase.getBool(firebaseData, path)) {
      if (firebaseData.dataType() == "boolean") {
        bool relayState = firebaseData.boolData();
        // Serial.println("Relay " + String(i) + ": " + String(relayState));

        switch (i) {
          case 1:
            doc_from_firebase["relayPin1"] = relayState ? 1 : 0;
            break;
          case 2:
            doc_from_firebase["relayPin2"] = relayState ? 1 : 0;
            break;
          case 3:
            doc_from_firebase["relayPin3"] = relayState ? 1 : 0;
            break;
          case 4:
            doc_from_firebase["relayPin4"] = relayState ? 1 : 0;
            break;
        }
      }
    } else {
      Serial.println("Failed to get relay " + String(i) + " value: " + firebaseData.errorReason());
      FirebaseJson json;
      switch (i) {
        case 1:
          doc_from_firebase["relayPin1"] = 0;
          break;
        case 2:
          doc_from_firebase["relayPin2"] = 0;
          break;
        case 3:
          doc_from_firebase["relayPin3"] = 0;
          break;
        case 4:
          doc_from_firebase["relayPin4"] = 0;
          break;
      }
      json.set(1, false);
      json.set(2, false);
      json.set(3, false);
      json.set(4, false);
      path =  "/devices/" + String(boardID) + "/relay/";
      if (Firebase.setJSON(firebaseData, path, json)) {
        Serial.println("Relay Data Sent Successfully.");
      } else {
        Serial.print("Failed to send data: ");
        Serial.println(firebaseData.errorReason());
      }
    }
  }
  // Serialize the JSON to send
  serializeJson(doc_from_firebase, send_jsondata);
  Serial.print("Data Sent: ");
  Serial.println(send_jsondata);
  Serial2.println(send_jsondata);
  send_jsondata = "";
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Connect To ESP

  // Connect to Wi-Fi
  connectWiFi(ssid1, password1);

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to the backup SSID...");
    connectWiFi(ssid2, password2);
  }

  // Firebase configuration
  firebaseConfig.api_key = API_KEY;
  firebaseConfig.database_url = DATABASE_URL;
  firebaseAuth.user.email = USER_EMAIL;
  firebaseAuth.user.password = USER_PASSWORD;

  Firebase.reconnectNetwork(true);
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  if (Firebase.ready()) {
    Serial.println("Firebase ready.");
  } else {
    Serial.println("Failed to initialize Firebase.");
  }
  Serial.println("Setup Completed");
}

void loop() {
  if (Serial2.available()) {
    recv_jsondata = Serial2.readStringUntil('\n');
    Serial.printf("Received Data: "); Serial.println(recv_jsondata);

    DeserializationError error = deserializeJson(doc_to_firebase, recv_jsondata);
    if (!error) {
      const char* datatype = doc_to_firebase["dataType"];
      int boardID = doc_to_firebase["boardID"];
      if (strcmp(datatype, "MAC") == 0) {
        uint8_t mac_addr[6];
        for (int i = 0; i < 6; i++) {
          mac_addr[i]=doc_to_firebase["MAC"][i];
        }
        char macStr[18];
        snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
        sentToFirebaseMAC(boardID, macStr);
      } else if (strcmp(datatype, "sensor") == 0) {
        float temperature = doc_to_firebase["temperature"];
        float humidity = doc_to_firebase["humidity"];
        float gas = doc_to_firebase["gas"];
        float pressure = doc_to_firebase["pressure"];
        float current = doc_to_firebase["current"];
        float voltage = doc_to_firebase["voltage"];
        sentToFirebase(boardID, temperature, humidity, gas, pressure, current, voltage);
        getFromFirebase(boardID);
      }
    } else {
      Serial.print("JSON Deserialization Error: ");
      Serial.println(error.c_str());
    }
    recv_jsondata = "";
    doc_to_firebase.clear();
    doc_from_firebase.clear();
  }
}
