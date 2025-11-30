// This program implements the logic for Project 3: Personalized Accessible Commute Companion.
// It features two modes: Full Power (tracking/GSM) and Low Power (monitoring/battery save).

// --- Library Includes ---
#include <TinyGPS++.h> // For GPS data parsing
#include <BLEDevice.h> // For Bluetooth Low Energy communication

// --- Configuration Constants ---
const char* TARGET_PHONE = "8072999767"; // Aptiv Mobility Advocate Team Contact
const char* SMS_ALERT_TEXT = "COMPANION EMERGENCY! Employee requires immediate help. Unit ID #203. ";

// Thresholds
const long POWER_DOWN_DELAY_MS = 5 * 60 * 1000; // 5 minutes of inactivity before low power mode
const int OBSTACLE_DISTANCE_CM = 80; // Obstacle distance threshold for Ultrasonic (80 cm)

// --- Pin Definitions ---
// Communication (UART)
#define GSM_RX_PIN 16 // Connect to GSM TX
#define GSM_TX_PIN 17 // Connect to GSM RX
#define GPS_RX_PIN 18 // Connect to GPS TX (Serial1)
#define GPS_TX_PIN 19 // Connect to GPS RX

// Sensor & Control Pins
#define RELAY_PIN 27 // Controls VCC for GSM/GPS modules
#define BUZZER_PIN 26
#define HALL_SENSOR_PIN 34 // User presence detection (Input)
#define PANIC_BUTTON_PIN 35 // User-triggered panic (Input)

// Ultrasonic Pins
#define TRIG_PIN 22
#define ECHO_PIN 23

// LED Status Pins (Used with 220-ohm Resistors)
#define LED_SYS_OK_PIN 4      // Green (System Running)
#define LED_NET_STATUS_PIN 5  // Yellow (GSM/GPS Active)
#define LED_ALERT_PIN 15      // Red (Critical Alert/Obstacle)

// BLE Service for Integration Protocol
#define BLE_SERVICE_UUID "2E4D1A0F-1234-4567-89AB-CDEF01234567"
#define BLE_CHAR_UUID    "7E4D1A0F-1234-4567-89AB-CDEF01234567"

// --- Global Variables ---
HardwareSerial SerialGPS(1); 
HardwareSerial SerialGSM(2); 
TinyGPSPlus gps;

BLEServer* pServer = NULL;
BLECharacteristic* pStatusCharacteristic = NULL;

typedef enum {
  LOW_POWER_MODE,
  FULL_POWER_MODE,
  EMERGENCY_MODE
} OperatingMode;

OperatingMode currentMode = LOW_POWER_MODE;
unsigned long lastActivityTime = 0;
bool isHallEngaged = false;
bool isPanicActive = false;

// --- Function Prototypes ---
void initPins();
void initBLE();
void initSerialCommunication();
void setMode(OperatingMode newMode);
void powerModules(bool enable);
void updateGPS();
void checkHallSensor();
void checkPanicButton();
void checkObstacle();
void sendSMSAlert(const String& message);
void setLedStatus(int pin, bool state);
void updateLeds();
long measureDistanceCM();
void cameraCapturePlaceholder(const char* context);

// =========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("--- Aptiv Commute Companion Initializing ---");

  initPins();
  initSerialCommunication();
  initBLE();
  
  // Start in low power mode to save battery immediately
  setMode(LOW_POWER_MODE);
  
  Serial.println("System Ready. Monitoring user presence...");
}

void loop() {
  checkPanicButton();
  checkHallSensor();

  if (currentMode == FULL_POWER_MODE || currentMode == EMERGENCY_MODE) {
    updateGPS();
    checkObstacle(); // Only check for obstacles when user is active
  }

  // Automatic power management check
  if (currentMode == FULL_POWER_MODE && (millis() - lastActivityTime > POWER_DOWN_DELAY_MS)) {
    Serial.println("Timeout reached. Switching to LOW POWER MODE.");
    setMode(LOW_POWER_MODE);
  }

  updateLeds();
  delay(100);
}

// =========================================================================
// --- Implementation Functions ---
// =========================================================================

void initPins() {
  // Setup Outputs
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_SYS_OK_PIN, OUTPUT);
  pinMode(LED_NET_STATUS_PIN, OUTPUT);
  pinMode(LED_ALERT_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  
  // Setup Inputs
  pinMode(HALL_SENSOR_PIN, INPUT_PULLDOWN);
  // Using INPUT_PULLUP for the panic button, assuming a button connecting pin to GND
  pinMode(PANIC_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ECHO_PIN, INPUT);

  // Initial state: Relay OFF (modules disabled)
  digitalWrite(RELAY_PIN, LOW); 
  digitalWrite(BUZZER_PIN, LOW);
}

void initSerialCommunication() {
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  SerialGSM.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN); 
}

void initBLE() {
  BLEDevice::init("Aptiv-Companion-Unit"); 

  pServer = BLEDevice::createServer();
  
  BLEService *pService = pServer->createService(BLE_SERVICE_UUID);
  
  pStatusCharacteristic = pService->createCharacteristic(
                                         BLE_CHAR_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY
                                       );
  pStatusCharacteristic->setValue("OFFLINE");
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  BLEDevice::startAdvertising();
  Serial.println("BLE Interface Active.");
}


void powerModules(bool enable) {
  // Use the Relay to enable/disable power to the GPS and GSM modules
  digitalWrite(RELAY_PIN, enable ? HIGH : LOW); 
  delay(2000); // Give modules time to power up/down
  Serial.printf("Modules Power %s\n", enable ? "ON" : "OFF");

  if (enable) {
    // Re-initialize GSM for text mode after power up
    SerialGSM.println("AT+CMGF=1"); 
    delay(100);
    // Notify BLE listeners that tracking is active
    pStatusCharacteristic->setValue("TRACKING_ACTIVE");
    pStatusCharacteristic->notify();
  } else {
    pStatusCharacteristic->setValue("LOW_POWER_IDLE");
    pStatusCharacteristic->notify();
  }
}

void setMode(OperatingMode newMode) {
  if (currentMode == newMode) return;
  
  currentMode = newMode;
  
  switch (currentMode) {
    case FULL_POWER_MODE:
      Serial.println("Mode: FULL POWER");
      powerModules(true);
      lastActivityTime = millis();
      break;
      
    case LOW_POWER_MODE:
      Serial.println("Mode: LOW POWER");
      powerModules(false);
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(LED_ALERT_PIN, LOW);
      break;
      
    case EMERGENCY_MODE:
      Serial.println("Mode: EMERGENCY");
      powerModules(true); // Ensure power is on for GSM/GPS
      // Alarm and SMS sent in trigger functions
      break;
  }
}

void checkPanicButton() {
  // The button connects the pin to GND when pressed
  if (digitalRead(PANIC_BUTTON_PIN) == LOW && !isPanicActive) {
    isPanicActive = true;
    Serial.println("--- PANIC BUTTON ACTIVATED! ---");
    setMode(EMERGENCY_MODE);
    cameraCapturePlaceholder("PANIC");

    // Attempt to get location and send alert
    updateGPS();
    if (gps.location.isValid() && gps.location.isUpdated()) {
      String location = String(gps.latitude(), 6) + "," + String(gps.longitude(), 6);
      sendSMSAlert(SMS_ALERT_TEXT + "Location: " + location + ". Reason: User Triggered.");
    } else {
      sendSMSAlert(SMS_ALERT_TEXT + "Location: Unavailable. Reason: User Triggered.");
    }
    digitalWrite(BUZZER_PIN, HIGH);
    
  } else if (digitalRead(PANIC_BUTTON_PIN) == HIGH && isPanicActive) {
    // Simple debouncing/reset mechanism
    delay(500); 
    if (digitalRead(PANIC_BUTTON_PIN) == HIGH) {
        Serial.println("Panic acknowledged or released.");
        isPanicActive = false;
        if(currentMode == EMERGENCY_MODE) {
            setMode(isHallEngaged ? FULL_POWER_MODE : LOW_POWER_MODE);
        }
    }
  }
}

void checkHallSensor() {
  bool currentHallState = (digitalRead(HALL_SENSOR_PIN) == HIGH);
  
  if (currentHallState && !isHallEngaged) {
    // Trip Started
    Serial.println("Hall Sensor Engaged. Trip Start detected.");
    isHallEngaged = true;
    setMode(FULL_POWER_MODE);
    cameraCapturePlaceholder("TRIP_START");
  } else if (!currentHallState && isHallEngaged) {
    // Trip Ended (Start Low Power timer)
    Serial.println("Hall Sensor Disengaged. Trip End detected. Starting Low Power timer...");
    isHallEngaged = false;
    lastActivityTime = millis();
    cameraCapturePlaceholder("TRIP_END");
  }

  if (isHallEngaged) {
      lastActivityTime = millis(); // Reset low power timer while user is detected
  }
}

void updateGPS() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

void checkObstacle() {
  long distance = measureDistanceCM();
  
  if (distance > 0 && distance < OBSTACLE_DISTANCE_CM) {
    Serial.printf("Obstacle Alert! Distance: %d cm\n", distance);
    setLedStatus(LED_ALERT_PIN, HIGH);
    digitalWrite(BUZZER_PIN, HIGH);
    
    // Optionally: Notify Advocacy Team (only for persistent or severe obstacles)
    // sendSMSAlert(SMS_ALERT_TEXT + "Location: " + String(gps.latitude(), 6) + ", " + String(gps.longitude(), 6) + ". Reason: Obstacle detected.");
  } else {
    // Clear local alert if not in full panic mode
    if (currentMode != EMERGENCY_MODE) {
      digitalWrite(BUZZER_PIN, LOW);
      setLedStatus(LED_ALERT_PIN, LOW);
    }
  }
}

long measureDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 20000); // 20ms timeout

  if (duration == 0) return -1;
  return duration / 58; 
}

void sendSMSAlert(const String& message) {
  if (currentMode == LOW_POWER_MODE) {
      Serial.println("Modules are OFF. Attempting temporary power-on for SMS...");
      powerModules(true); // Temporarily power on
  }
  
  Serial.println("Sending SMS...");
  SerialGSM.print("AT+CMGS=\"");
  SerialGSM.print(TARGET_PHONE);
  SerialGSM.println("\"");
  delay(100);
  SerialGSM.print(message);
  SerialGSM.write(0x1A); // Ctrl+Z to send
  Serial.println("SMS command sent.");

  if (currentMode == LOW_POWER_MODE) {
      delay(5000); // Wait for transmission to complete
      powerModules(false); // Power off again
  }
}

void cameraCapturePlaceholder(const char* context) {
  // The full implementation would use a camera module (e.g., ESP32-CAM)
  // to capture the immediate environment and upload the image via GSM/Wi-Fi/BLE.
  Serial.printf("-> DOCUMENTATION: Capturing image for context: %s\n", context);
}

void setLedStatus(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

void updateLeds() {
  // Green LED: Always ON when system is running
  setLedStatus(LED_SYS_OK_PIN, HIGH); 

  // Yellow LED: GSM/GPS/BLE active (Full Power or Emergency)
  bool networkActive = (currentMode == FULL_POWER_MODE || currentMode == EMERGENCY_MODE);
  setLedStatus(LED_NET_STATUS_PIN, networkActive);

  // Red LED: Blinks for panic or stays solid for obstacle warning
  if (currentMode == EMERGENCY_MODE || isPanicActive) {
      // Fast blink for emergency
      static unsigned long lastRedBlinkTime = 0;
      if (millis() - lastRedBlinkTime > 150) {
          digitalWrite(LED_ALERT_PIN, !digitalRead(LED_ALERT_PIN));
          lastRedBlinkTime = millis();
      }
  } else if (digitalRead(BUZZER_PIN) == HIGH) {
      // Solid ON for local obstacle warning
      setLedStatus(LED_ALERT_PIN, HIGH);
  } else {
      setLedStatus(LED_ALERT_PIN, LOW);
  }
}
