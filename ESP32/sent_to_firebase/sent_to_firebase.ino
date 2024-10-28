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
const char* ssid = "GNXS-2.4G-31C3F0";
const char* password = "25051971";

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

// Function to send data to Firebase
void sendData(SensorData data) {
  Serial.printf("Classroom ID: %d :- Temperature: %.2f, Humidity: %.2f, Air Quality: %.2f, Pressure: %.2f, Current: %.2f, Voltage: %.2f\n", 
                  data.boardId, data.temperature, data.humidity, data.gas, data.pressure, data.current, data.voltage);
  String path = "/devices" + String(data.boardId) + "/sensor";

  if (Firebase.setFloat(firebaseData, path + "/temperature", data.temperature)) {
    Serial.println("Temperature sent successfully");
  } else {
    Serial.println("Failed to send temperature: " + firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, path + "/humidity", data.humidity)) {
    Serial.println("Humidity sent successfully");
  } else {
    Serial.println("Failed to send humidity: " + firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, path + "/gas", data.gas)) {
    Serial.println("Gas sent successfully");
  } else {
    Serial.println("Failed to send gas: " + firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, path + "/pressure", data.pressure)) {
    Serial.println("Pressure sent successfully");
  } else {
    Serial.println("Failed to send pressure: " + firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, path + "/current", data.current)) {
    Serial.println("Current sent successfully");
  } else {
    Serial.println("Failed to send current: " + firebaseData.errorReason());
  }

  if (Firebase.setFloat(firebaseData, path + "/voltage", data.voltage)) {
    Serial.println("Voltage sent successfully");
  } else {
    Serial.println("Failed to send voltage: " + firebaseData.errorReason());
  }
}

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600, SERIAL_8N1, RXD2, TXD2); // Connect To ESP

    // Connect to Wi-Fi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
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
    Serial.printf("Classroom ID: %d :- Temperature: %.2f, Humidity: %.2f, Air Quality: %.2f, Pressure: %.2f, Current: %.2f, Voltage: %.2f\n", 
                  receivedData.boardId, receivedData.temperature, receivedData.humidity, receivedData.gas, receivedData.pressure, receivedData.current, receivedData.voltage);
    
    // Send the received data to Firebase
    sendData(receivedData);
  } else {
    Serial.println("No data received");
  }
}
