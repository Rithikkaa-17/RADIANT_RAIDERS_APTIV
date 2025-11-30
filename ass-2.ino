// This program implements the logic for Project 2: Accessible Stop/Route Identification Assistant.
// It verifies vehicle ID (RFID), confirms parking (Hall Sensor), checks safety (Ultrasonic), 
// and notifies the waiting employee via Bluetooth.

// --- Library Includes ---
#include <SPI.h> // Required for RFID communication
#include <MFRC522.h> // RFID Reader Library
#include <BLEDevice.h> // ESP32 Bluetooth Low Energy (BLE) Library

// --- Configuration Constants ---
// Define the specific RFID tag ID (UID) of the Aptiv Accessible Shuttle
// This is a dummy UID. REPLACE with the actual UID of your test tag (e.g., {0xDE, 0xAD, 0xBE, 0xEF})
constexpr byte APTIV_SHUTTLE_UID[] = {0xA1, 0xB2, 0xC3, 0xD4}; 
const int UID_SIZE = sizeof(APTIV_SHUTTLE_UID);

// BLE Service UUID for the MobilityMakers App integration protocol
#define BLE_SERVICE_UUID "4FAF062C-0994-44C5-973D-6842D3D3F12C"
#define BLE_CHAR_UUID    "E376A96E-A54B-4E7B-902B-2900A9E7796D"

// Ultrasonic Thresholds
const int SAFETY_DISTANCE_CM = 100; // Minimum clear distance for ramp deployment (1 meter)
const int ULTRASONIC_TIMEOUT_US = 20000; // Timeout for 20ms

// --- Pin Definitions ---
// RFID Pins
#define RST_PIN 21 // RFID Reset
#define SS_PIN  5  // RFID Slave Select (SDA)

// Ultrasonic Pins
#define TRIG_PIN 22
#define ECHO_PIN 23

// Sensor Pins
#define HALL_SENSOR_PIN 34 // Input from Hall Sensor

// Output Pins
#define RELAY_PIN 27 // Accessible Stop Beacon/Light
#define LED_RED_PIN 17 // Status: Wait/Error
#define LED_YELLOW_PIN 16 // Status: Checking
#define LED_GREEN_PIN 4 // Status: Verified/Clear

// --- Global Variables ---
MFRC522 mfrc522(SS_PIN, RST_PIN); // Create MFRC522 instance
BLEServer* pServer = NULL;
BLECharacteristic* pStatusCharacteristic = NULL;
bool deviceConnected = false;

// --- Function Prototypes ---
void initPins();
void initBLE();
void setStatusLED(int colorPin, bool state);
bool readRFIDTag();
bool compareUIDs(byte* buffer1, byte* buffer2, int size);
long measureDistanceCM();
void captureStillImage();
void processArrival(const String& shuttleID);
void resetStatus();

// =========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("--- Aptiv Accessible Stop Assistant Initializing ---");

  initPins();
  
  // Initialize RFID reader
  SPI.begin();
  mfrc522.PCD_Init();
  Serial.println("RFID Reader Initialized.");

  initBLE();
  
  // Initial Status Check: Red and Yellow briefly on
  setStatusLED(LED_RED_PIN, HIGH);
  setStatusLED(LED_YELLOW_PIN, HIGH);
  delay(500);
  setStatusLED(LED_RED_PIN, LOW);
  setStatusLED(LED_YELLOW_PIN, LOW);
  
  Serial.println("System Ready. Waiting for Vehicle...");
}

void loop() {
  // 1. Wait for Vehicle Presence (Hall Sensor Check)
  // Hall Sensor goes HIGH when a magnet (on the vehicle) is in position
  if (digitalRead(HALL_SENSOR_PIN) == HIGH) {
    setStatusLED(LED_YELLOW_PIN, HIGH); // Yellow: Checking Status
    Serial.println("Vehicle Detected (Hall Sensor Triggered). Starting Verification Sequence...");
    
    // 2. Verification Sequence
    if (readRFIDTag()) {
      Serial.println("RFID Verified. Initiating Safety Check.");
      
      // 3. Safety Check
      long distance = measureDistanceCM();
      if (distance > 0 && distance >= SAFETY_DISTANCE_CM) {
        Serial.printf("Safety Check SUCCESS: Path clear (Distance: %d cm)\n", distance);
        
        // 4. Successful Arrival Process
        processArrival("AptivShuttle-101"); 
        
        // Wait until the vehicle moves away
        while (digitalRead(HALL_SENSOR_PIN) == HIGH) {
          delay(1000); // Wait in success state
        }
        
        Serial.println("Vehicle departed. Resetting system.");
        resetStatus();
        
      } else {
        Serial.printf("Safety Check FAIL: Obstruction detected (Distance: %d cm). Waiting for clearance.\n", distance);
        setStatusLED(LED_RED_PIN, HIGH);
        // Alert dispatch or driver (via BLE or a local display, if added)
        delay(5000); // Wait and retry/re-check
      }
      
    } else {
      Serial.println("RFID FAIL: Unknown or Unconfirmed Shuttle. Alerting.");
      setStatusLED(LED_RED_PIN, HIGH);
      // Wait for the vehicle to move (Hall Sensor to go LOW)
      while (digitalRead(HALL_SENSOR_PIN) == HIGH) {
        delay(500); 
      }
      Serial.println("Vehicle moved. Resetting.");
      resetStatus();
    }
  } else {
    // Idle state
    setStatusLED(LED_YELLOW_PIN, LOW);
    setStatusLED(LED_RED_PIN, LOW);
    setStatusLED(LED_GREEN_PIN, LOW);
    digitalWrite(RELAY_PIN, LOW); // Ensure beacon is off
    delay(50);
  }
}

// =========================================================================
// --- Implementation Functions ---
// =========================================================================

void initPins() {
  pinMode(HALL_SENSOR_PIN, INPUT_PULLDOWN); // Use pulldown for Hall sensor
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
}

void initBLE() {
  BLEDevice::init("MobilityMakers_Stop_Assistant"); // Name for the BLE device

  pServer = BLEDevice::createServer();
  
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  
  pStatusCharacteristic = pService->createCharacteristic(
                                         BLE_CHAR_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pStatusCharacteristic->setValue("WAITING_VEHICLE"); // Initial status
  
  pService->start();

  // Start advertising the service so the mobile app can discover it
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06); // Min interval
  pAdvertising->setMinPreferred(0x12); // Max interval
  BLEDevice::startAdvertising();
  Serial.println("BLE Advertising Started. Ready for Mobile Connection.");
}

void setStatusLED(int colorPin, bool state) {
  // Ensure only one primary status LED is ON at a time for clarity
  if (state == HIGH) {
    if (colorPin != LED_RED_PIN) digitalWrite(LED_RED_PIN, LOW);
    if (colorPin != LED_YELLOW_PIN) digitalWrite(LED_YELLOW_PIN, LOW);
    if (colorPin != LED_GREEN_PIN) digitalWrite(LED_GREEN_PIN, LOW);
  }
  digitalWrite(colorPin, state);
}

// Attempts to read an RFID tag and compare it to the known shuttle UID
bool readRFIDTag() {
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    Serial.print("Tag UID found: ");
    String currentUID = "";
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      currentUID += String(mfrc522.uid.uidByte[i], HEX);
      Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " ");
      Serial.print(mfrc522.uid.uidByte[i], HEX);
    }
    Serial.println();
    
    // Compare the detected UID with the target shuttle UID
    if (compareUIDs(mfrc522.uid.uidByte, (byte*)APTIV_SHUTTLE_UID, UID_SIZE)) {
      mfrc522.PICC_HaltA(); // Stop reading current card
      return true;
    }
    mfrc522.PICC_HaltA(); 
  }
  return false;
}

// Compare two byte arrays (UIDs)
bool compareUIDs(byte* buffer1, byte* buffer2, int size) {
  for (int i = 0; i < size; i++) {
    if (buffer1[i] != buffer2[i]) {
      return false;
    }
  }
  return true;
}

// Measures distance using the Ultrasonic Sensor
long measureDistanceCM() {
  // Clear the trigger pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  
  // Set the trigger pin high for 10 microseconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read the echo pin, returns the sound wave travel time in microseconds
  long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

  // Calculate distance in cm (Speed of sound = 340 m/s or 29 microseconds per cm)
  // Distance = Duration / 58
  if (duration == 0) return -1; // Timeout/No measurement
  return duration / 58; 
}

// Placeholder for the Camera function
void captureStillImage() {
  // In a full implementation using a CAM module (like OV2640), this function
  // would capture an image and attempt to save it to an SD card or upload it
  // over Wi-Fi/Bluetooth to the server for documentation.
  Serial.println("-> DOCUMENTATION: Capturing image of stop condition...");
  // Simulate processing time
  delay(1000); 
  Serial.println("-> Image capture simulated.");
}

void processArrival(const String& shuttleID) {
  setStatusLED(LED_GREEN_PIN, HIGH); // Green: Success
  digitalWrite(RELAY_PIN, HIGH);      // Activate beacon/light
  
  // Update BLE status for employee notification
  String statusMessage = "ARRIVED:";
  statusMessage += shuttleID;
  pStatusCharacteristic->setValue(statusMessage.c_str());
  pStatusCharacteristic->notify();
  Serial.printf("BLE Notification Sent: %s\n", statusMessage.c_str());

  // Take documentation snapshot
  captureStillImage(); 
}

void resetStatus() {
  setStatusLED(LED_GREEN_PIN, LOW);
  digitalWrite(RELAY_PIN, LOW);
  
  pStatusCharacteristic->setValue("WAITING_VEHICLE");
  pStatusCharacteristic->notify();
  
  Serial.println("Status reset.");
}