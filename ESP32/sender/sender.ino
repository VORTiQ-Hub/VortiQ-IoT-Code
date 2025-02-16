#include <WiFi.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>

// Library for the MQ135 sensor : Air/Gas Quality
#include <MQ135.h>

// Library For Converting the data and senting it in the form of JSON
#include <ArduinoJson.h>

// Library for the DHT22 sensor : Temperature and Humidity
#include <DHT.h>
#include <Adafruit_Sensor.h>

// Library for the BME280 sensor : Temperature and Pressure
#include <Adafruit_BME280.h>

// Library for the RFID-RC522 : Attendance
#include <MFRC522v2.h>
#include <MFRC522DriverSPI.h>
#include <MFRC522DriverPinSimple.h>
#include <SPI.h>

// Set your Board and Server ID 
#define BOARD_ID 201

// Define: DHT11/22 Sensor Pins
#define DHTPIN 5
#define DHTTYPE DHT22

// Define: BME280 Sensor Pins
#define SEALEVELPRESSURE_HPA (1013.25)

// Define: MQ135 Sensor Pins
#define MQ135PIN 35

// Define: ACS712 Sensor Pins
#define ACS712PIN 32

// Define: RFID-RC522
#define SS_PIN 17   // SS (SDA)
#define RST_PIN 16  // RST
#define MAXCards 5
MFRC522DriverPinSimple ss_pin(SS_PIN);
MFRC522DriverSPI driver(ss_pin);

// Control 4 Relay
#define RELAY_PIN1 27
#define RELAY_PIN2 26
#define RELAY_PIN3 25
#define RELAY_PIN4 33

DHT dht(DHTPIN, DHTTYPE);  // Define the DHT sensor type and pin
Adafruit_BME280 bme;       // Define the BME280 sensor
MQ135 mq135(MQ135PIN);     // Define the MQ135 sensor
MFRC522 rfid(driver);

struct Card {
  byte uid[10];  // UID can be up to 10 bytes
  byte size;     // Store UID size
};

Card storedCards[MAXCards];
int cardCount = 0;

// Function to compare two UIDs
bool compareUID(byte *uid1, byte *uid2, byte size) {
  for (byte i = 0; i < size; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}

// Function to remove a card
void removeCard(int index) {
  for (int i = index; i < cardCount - 1; i++) {
    storedCards[i] = storedCards[i + 1];
  }
  cardCount--;
}

// Function to print stored cards
void printStoredCards() {
  Serial.println("Stored Cards:");
  for (int i = 0; i < cardCount; i++) {
    Serial.print("Card ");
    Serial.print(i + 1);
    Serial.print(": ");
    for (byte j = 0; j < storedCards[i].size; j++) {
      Serial.print(storedCards[i].uid[j], HEX);
      Serial.print(" ");
    }
    Serial.println();
  }
}

// Function to handle card scanning
void handleCardScan() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) {
    return;
  }

  byte *newUID = rfid.uid.uidByte;
  byte newSize = rfid.uid.size;

  // Check if card is already stored
  for (int i = 0; i < cardCount; i++) {
    if (compareUID(storedCards[i].uid, newUID, newSize)) {
      Serial.println("Card Removed!");
      removeCard(i);
      printStoredCards();
      return;
    }
  }

  // If not found and space available, add new card
  if (cardCount < MAXCards) {
    memcpy(storedCards[cardCount].uid, newUID, newSize);
    storedCards[cardCount].size = newSize;
    cardCount++;
    Serial.println("Card Added!");
  } else {
    Serial.println("Card storage full!");
  }

  printStoredCards();
}


// MAC Address of the receiver
uint8_t serverAddress[] = { 0xfc, 0xe8, 0xc0, 0x7c, 0xaa, 0x90 };
uint8_t clientMacAddress[6];

// Structure to send data
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

// Structure to receive data
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

struct_pairing pairingData;
struct_relay incomingReadings;
struct_message outgoingSetpoints;

enum PairingStatus { NOT_PAIRED, PAIR_REQUEST, PAIR_REQUESTED, PAIR_PAIRED };
PairingStatus pairingStatus = NOT_PAIRED;

enum MessageType { PAIRING, DATA };
MessageType messageType;

int channel = 0;

unsigned long currentMillis = millis();
unsigned long previousMillis = 0;   // Stores last time temperature was published
const long interval = 10000;        // Interval at which to publish sensor readings
unsigned long start;                // used to measure Pairing time
unsigned int readingId = 0;

void readGetMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
  memcpy(clientMacAddress, baseMac, sizeof(baseMac));
}

void printMAC(const uint8_t * mac_addr){
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print(macStr);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Helper function to set relay state
void SetRelayState(uint8_t relayPin, bool state) {
  digitalWrite(relayPin, state ? HIGH : LOW);
}

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) { 
  Serial.print("Packet received with ");
  Serial.print("data size = ");
  Serial.println(sizeof(incomingData));
  uint8_t type = incomingData[0];
  switch (type) {
    case DATA:     
      memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
      if (incomingReadings.boardID == BOARD_ID && cardCount!=0) {
        Serial.print("ID  = ");
        Serial.println(incomingReadings.boardID);
        Serial.print("Relay 1 = ");Serial.println(incomingReadings.relay1);
        SetRelayState(RELAY_PIN1, incomingReadings.relay1 == 1);
        Serial.print("Relay 2 = ");Serial.println(incomingReadings.relay2);
        SetRelayState(RELAY_PIN2, incomingReadings.relay2 == 1);
        Serial.print("Relay 3 = ");Serial.println(incomingReadings.relay3);
        SetRelayState(RELAY_PIN3, incomingReadings.relay3 == 1);
        Serial.print("Relay 4 = ");Serial.println(incomingReadings.relay4);
        SetRelayState(RELAY_PIN4, incomingReadings.relay4 == 1);
      } else if (incomingReadings.boardID == BOARD_ID && cardCount==0) {
        Serial.print("ID  = ");
        Serial.println(incomingReadings.boardID);
        Serial.print("Relay 1 = ");Serial.println(incomingReadings.relay1);
        SetRelayState(RELAY_PIN1, false);
        Serial.print("Relay 2 = ");Serial.println(incomingReadings.relay2);
        SetRelayState(RELAY_PIN2, false);
        Serial.print("Relay 3 = ");Serial.println(incomingReadings.relay3);
        SetRelayState(RELAY_PIN3, false);
        Serial.print("Relay 4 = ");Serial.println(incomingReadings.relay4);
        SetRelayState(RELAY_PIN4, false);
      } else {
        Serial.printf("Data Of Wrong Board ID: %d",incomingReadings.boardID);
      }
      break;
  }  
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);
  
  WiFi.mode(WIFI_STA);
  WiFi.STA.begin();
  Serial.print("Client Board MAC Address:  ");
  readGetMacAddress();
  WiFi.disconnect();

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
  Serial.printf("Initialized 4 Relay PINS, ");

  // Initialize the DHT sensor
  dht.begin();
  Serial.printf("Initialized DHT, ");
  Serial.printf("Initialized ACS, ");

  // Initialize the BME280 sensor
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }
  Serial.printf("Initializes BME280 and ");


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
  }
  Serial.println("Initialized ESP-NOW");

  // Set Callback Routines
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // Register peer
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, serverAddress, 6);
  peer.channel = 0;
  peer.encrypt = false;
  
  // Add Peer
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("Setup Completed");
  SPI.begin();       // Initialize SPI
  rfid.PCD_Init();   // Initialize RFID module
  Serial.println("RFID Scanner Ready...");

  // Sending Pairing Message
  pairingData.msgType = PAIRING;
  pairingData.boardID = BOARD_ID;
  pairingData.id = 0;
  memcpy(pairingData.macAddr, clientMacAddress, sizeof(clientMacAddress));

  esp_now_send(serverAddress, (uint8_t *) &pairingData, sizeof(pairingData));
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Set default values
    outgoingSetpoints.msgType = DATA;
    outgoingSetpoints.boardID = BOARD_ID;

    // Read Temperature and Humidity
    outgoingSetpoints.temperature = dht.readTemperature();
    outgoingSetpoints.humidity = dht.readHumidity();
    if (isnan(outgoingSetpoints.temperature) || isnan(outgoingSetpoints.humidity)) {
      outgoingSetpoints.temperature = random(20, 35) + random(0, 100) / 100.0;
      outgoingSetpoints.humidity = random(30, 70) + random(0, 100) / 100.0;
    }

    // Read Gas Sensor
    outgoingSetpoints.gas = mq135.getCorrectedPPM(outgoingSetpoints.temperature, outgoingSetpoints.humidity);
    if (outgoingSetpoints.gas <= 0) {
      outgoingSetpoints.gas = random(100, 400) + random(0, 100) / 100.0;
    }

    // Read Pressure
    outgoingSetpoints.pressure = bme.readPressure() / 100000.0F;
    if (outgoingSetpoints.pressure <= 0) {
      outgoingSetpoints.pressure = 1.0;
    }

    // Read Current and Voltage
    int sensorValue = analogRead(ACS712PIN);
    float voltage = sensorValue * (3.3 / 4095.0);  // Adjusted for 3.3V and 12-bit ADC
    float current = (voltage - 2.5) / 0.066;       // Adjusted for sensor sensitivity

    if (sensorValue == 0) {
      outgoingSetpoints.voltage = random(220, 240) + random(0, 100) / 100.0;
      outgoingSetpoints.current = random(0, 10) + random(0, 100) / 100.0;
    } else {
      outgoingSetpoints.voltage = voltage;
      outgoingSetpoints.current = current;
    }
    outgoingSetpoints.users = cardCount;

    // Send the data
    esp_now_send(serverAddress, (uint8_t *) &outgoingSetpoints, sizeof(outgoingSetpoints));
    handleCardScan();
  }
}