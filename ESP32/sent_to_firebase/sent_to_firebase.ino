#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <HardwareSerial.h>
#include <ArduinoJson.h>

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
StaticJsonDocument<256> doc_from_central;
StaticJsonDocument<256> doc_to_central;

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

void sentToFirebase(int boardId, float temperature, float humidity, float airQuality, float pressure, float current, float voltage) {
  String path = "/devices/" + String(boardId) + "/sensor";
  FirebaseJson json;
  json.set("temperature", temperature);
  json.set("humidity", humidity);
  json.set("airQuality", airQuality);
  json.set("pressure", pressure);
  json.set("current", current);
  json.set("voltage", voltage);
  if (Firebase.setJSON(firebaseData, path, json)) {
    Serial.println("Data sent successfully.");
  } else {
    Serial.print("Failed to send data: ");
    Serial.println(firebaseData.errorReason());
  }
}

void getFromFirebase(int boardId, const char* macAddress) {
  doc_to_central["boardId"] = boardId;
  doc_to_central["macAddress"] = macAddress;
  for(int i=1;i<=4;i++) {
    String path = "/devices/" + String(boardId) + "/relay/" + String(i);
    
    if (Firebase.getBool(firebaseData, path)) {
      if (firebaseData.dataType() == "boolean") {
        bool relayState = firebaseData.boolData();
        // Serial.println("Relay " + String(i) + ": " + String(relayState));
        
        switch (i) {
          case 1:
            doc_to_central["relayPin1"] = relayState ? true : false;
            break;
          case 2:
            doc_to_central["relayPin2"] = relayState ? true : false;
            break;
          case 3:
            doc_to_central["relayPin3"] = relayState ? true : false;
            break;
          case 4:
            doc_to_central["relayPin4"] = relayState ? true : false;
            break;
        }
      }
    } else {
      Serial.println("Failed to get relay " + String(i) + " value: " + firebaseData.errorReason());
    }
  }
  // Serialize the JSON to send
  Serial.print("Serilising To Serial2: ");
  serializeJson(doc_to_central, send_jsondata);
  Serial2.println(send_jsondata);
  send_jsondata = "";
  doc_to_central.clear();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);  // Connect To ESP

  // Connect to Wi-Fi
  // Attempt to connect to ssid1 first
  connectWiFi(ssid1, password1);

  // If ssid1 is unavailable, attempt to connect to ssid2
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to the backup SSID...");
    connectWiFi(ssid2, password2);
  }

  // Firebase configuration
  firebaseConfig.api_key = API_KEY;            // Set the API key
  firebaseConfig.database_url = DATABASE_URL;  // Add database URL
  firebaseAuth.user.email = USER_EMAIL;        // Set user email
  firebaseAuth.user.password = USER_PASSWORD;  // Set user password

  // Initialize Firebase with user authentication
  Firebase.reconnectNetwork(true);  // Reconnect automatically
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  // Check if Firebase is ready
  if (Firebase.ready()) {
    String base_path = "/devices";
    String var = "$userId";
    String val = "($userId === auth.uid)";
    Firebase.setReadWriteRules(firebaseData, base_path, var, val, val, DATABASE_SECRET);
    Serial.println("Success to initialize Firebase");
  } else {
    Serial.println("Failed to initialize Firebase.");
  }
  Serial.println("Setup Completed");
}

void loop() {
  if (Serial2.available()) {
    recv_jsondata = Serial2.readStringUntil('\n');
    Serial.println(recv_jsondata);

    DeserializationError error = deserializeJson(doc_from_central, recv_jsondata);
    if (!error) {
      int boardId = doc_from_central["boardId"];
      float temperature = doc_from_central["temperature"];
      float humidity = doc_from_central["humidity"];
      float airQuality = doc_from_central["airQuality"];
      float pressure = doc_from_central["pressure"];
      float current = doc_from_central["current"];
      float voltage = doc_from_central["voltage"];
      const char* macAddress = doc_from_central["macAddress"];

      sentToFirebase(boardId, temperature, humidity, airQuality, pressure, current, voltage);
      getFromFirebase(boardId, macAddress);
    }
    recv_jsondata = "";
    doc_from_central.clear();
  }
}
