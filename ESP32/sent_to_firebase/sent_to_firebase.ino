#include <WiFi.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <HardwareSerial.h>

// Connection To ESP
#define RXD2 16
#define TXD2 17
HardwareSerial mySerial(2); 

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

SensorData receivedData;

// Structure to receive data
struct RelayData {
  int boardId;
  bool relay1;
  bool relay2;
  bool relay3;
  bool relay4;
};

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

// Function to send data to & fetch data from Firebase 
void sendData(SensorData data) {
  Serial.printf("Classroom ID: %d :- Temperature: %.2f, Humidity: %.2f, Air Quality: %.2f, Pressure: %.2f, Current: %.2f, Voltage: %.2f\n", data.boardId, data.temperature, data.humidity, data.gas, data.pressure, data.current, data.voltage);
  String path = "/devices/" + String(data.boardId);

  // Sent To Firebase
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/temperature", data.temperature) ? "Temperature sent successfully" : "Failed to send temperature: " + firebaseData.errorReason());
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/humidity", data.humidity) ? "Humidity sent successfully" : "Failed to send humidity: " + firebaseData.errorReason());
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/gas", data.gas) ? "Gas sent successfully" : "Failed to send gas: " + firebaseData.errorReason());
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/pressure", data.pressure) ? "Pressure sent successfully" : "Failed to send pressure: " + firebaseData.errorReason());
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/current", data.current) ? "Current sent successfully" : "Failed to send current: " + firebaseData.errorReason());
  Serial.println(Firebase.setFloat(firebaseData, path + "/sensor/voltage", data.current) ? "Voltage sent successfully" : "Failed to send voltage: " + firebaseData.errorReason());

  // Receive From Firebase
  RelayData relay; 
  relay.boardId = data.boardId;
  if (Firebase.getBool(firebaseData, path + "/relay/1")) {
    relay.relay1 = firebaseData.boolData();
  } else {
    Serial.println("Failed to get relay 1 value: " + firebaseData.errorReason());
  }
  if (Firebase.getBool(firebaseData, path + "/relay/2")) {
    relay.relay2 = firebaseData.boolData();
  } else {
    Serial.println("Failed to get relay 2 value: " + firebaseData.errorReason());
  }
  if (Firebase.getBool(firebaseData, path + "/relay/3")) {
    relay.relay3 = firebaseData.boolData();
  } else {
    Serial.println("Failed to get relay 3 value: " + firebaseData.errorReason());
  }
  if (Firebase.getBool(firebaseData, path + "/relay/4")) {
    relay.relay4 = firebaseData.boolData();
  } else {
    Serial.println("Failed to get relay 4 value: " + firebaseData.errorReason());
  }
  mySerial.write(((uint8_t*)&relay), sizeof(relay));
  delay(2000);

}

void setup() {
  Serial.begin(115200);
  mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2); // Connect To ESP

  // Connect to Wi-Fi
  // Attempt to connect to ssid1 first
  connectWiFi(ssid1, password1);

  // If ssid1 is unavailable, attempt to connect to ssid2
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Attempting to connect to the backup SSID...");
    connectWiFi(ssid2, password2);
  }
  Serial.println("Connected to WiFi!");

  // Firebase configuration
  firebaseConfig.api_key = API_KEY;              // Set the API key
  firebaseConfig.database_url = DATABASE_URL;    // Add database URL
  firebaseAuth.user.email = USER_EMAIL;          // Set user email
  firebaseAuth.user.password = USER_PASSWORD;    // Set user password

  // Initialize Firebase with user authentication
  Firebase.reconnectNetwork(true);  // Reconnect automatically
  Firebase.begin(&firebaseConfig, &firebaseAuth);

  // Check if Firebase is ready
  if (Firebase.ready()) {
    String base_path = "/devices";
    String var = "$userId";
    String val = "($userId === auth.uid)";
    Firebase.setReadWriteRules(firebaseData, base_path, var, val, val, DATABASE_SECRET);
    
    // Initialize received data
    receivedData.boardId = 0;
    receivedData.temperature = 52;
    receivedData.humidity = 52;
    receivedData.gas = 52;
    receivedData.pressure = 52;
    receivedData.current = 58;
    receivedData.voltage = 255;
    sendData(receivedData);
  } else {
    Serial.println("Failed to initialize Firebase.");
  }
}

void loop() {
  if (mySerial.available() > 8) {
    Serial.println("Data available on Serial");
    mySerial.readBytes((char*)&receivedData, sizeof(receivedData));
    sendData(receivedData);
  } 

  delay(100);
}
