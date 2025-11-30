// --- Configuration Constants ---
// Define the phone number and alert message for emergency communication
const char* TARGET_PHONE = "8072999767"; // REPLACE with the Aptiv Dispatch Number
const char* SMS_ALERT_TEXT = "CRITICAL SAFETY ALERT! Check GPS location for Aptiv Unit ID #101. ";

// --- Pin Definitions ---
// GPS/GSM communication ports (using hardware serial on ESP32)
#define GSM_RX_PIN 16 // Connect to GSM TX
#define GSM_TX_PIN 17 // Connect to GSM RX
#define GPS_RX_PIN 18 // Connect to GPS TX (Using Serial1)
#define GPS_TX_PIN 19 // Connect to GPS RX

// Sensor Pins
#define SMOKE_SENSOR_PIN 34   // Analog Input (ADC1_CH6)
#define VIBRATION_SENSOR_PIN 25 // Digital Input (Alert on HIGH)

// Output Pins
#define BUZZER_PIN 26
#define RELAY_PIN 27

// LED Status Pins (Used with 220-ohm Resistors)
#define LED_SYS_OK_PIN 4      // Green (System Running)
#define LED_NET_STATUS_PIN 5  // Yellow (GSM Connected)
#define LED_ALERT_PIN 15      // Red (Critical Alert)

// --- Global Variables ---
// Hardware Serial ports for GPS and GSM
HardwareSerial SerialGPS(1); // Using Serial 1
HardwareSerial SerialGSM(2); // Using Serial 2

// GPS Object
TinyGPSPlus gps;

// Alert Status Flags
bool isAlertActive = false;
unsigned long lastAlertTime = 0;
const long ALERT_INTERVAL_MS = 60000; // Send new alert every 60 seconds

// Sensor Thresholds
const int SMOKE_THRESHOLD = 500; // Analog value threshold (0-4095). Adjust based on testing.

// --- Function Prototypes ---
void initPins();
void initSerialCommunication();
void updateGPS();
void checkSensors();
void triggerAlert(const char* reason);
void sendSMSAlert(const String& message);
void setLedStatus(int pin, bool state);
void pulseBuzzer();
void updateSystemStatusLeds();

// =========================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("--- Aptiv MobilityMakers Vehicle Safety Unit Initializing ---");

  initPins();
  initSerialCommunication();
  
  // Initial system check indicator
  for (int i = 0; i < 3; i++) {
    setLedStatus(LED_SYS_OK_PIN, HIGH);
    delay(100);
    setLedStatus(LED_NET_STATUS_PIN, HIGH);
    delay(100);
    setLedStatus(LED_ALERT_PIN, HIGH);
    delay(100);
    setLedStatus(LED_SYS_OK_PIN, LOW);
    setLedStatus(LED_NET_STATUS_PIN, LOW);
    setLedStatus(LED_ALERT_PIN, LOW);
  }
}

void loop() {
  // 1. Update GPS Data
  updateGPS();

  // 2. Check for critical safety events
  checkSensors();

  // 3. Manage repeated alerts if the condition persists
  if (isAlertActive && (millis() - lastAlertTime > ALERT_INTERVAL_MS)) {
    Serial.println("Re-sending critical alert SMS...");
    String fullMessage = SMS_ALERT_TEXT;
    fullMessage += "Status: Ongoing. Location: " + String(gps.latitude(), 6) + ", " + String(gps.longitude(), 6);
    sendSMSAlert(fullMessage);
    lastAlertTime = millis();
  }

  // 4. Update visual status for the driver
  updateSystemStatusLeds();

  delay(100); // Small delay to prevent watchdog timer from tripping
}

// =========================================================================
// --- Implementation Functions ---
// =========================================================================

void initPins() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW); // Start off
  
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Start off (Hazard Lights Off)

  pinMode(VIBRATION_SENSOR_PIN, INPUT); // Vibration sensor digital pin

  pinMode(LED_SYS_OK_PIN, OUTPUT);
  pinMode(LED_NET_STATUS_PIN, OUTPUT);
  pinMode(LED_ALERT_PIN, OUTPUT);
  
  digitalWrite(LED_SYS_OK_PIN, HIGH); // System is running
  digitalWrite(LED_NET_STATUS_PIN, LOW);
  digitalWrite(LED_ALERT_PIN, LOW);
}

void initSerialCommunication() {
  // GPS module uses standard 9600 baud rate
  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial initialized on pins 18/19.");
  
  // GSM module uses 9600 or 115200 baud rate. Start low, then auto-detect if necessary.
  SerialGSM.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN); 
  Serial.println("GSM Serial initialized on pins 16/17.");

  // Test GSM connectivity by sending AT command
  SerialGSM.println("AT");
  delay(100);
  if (SerialGSM.available()) {
    String response = SerialGSM.readString();
    if (response.indexOf("OK") != -1) {
      Serial.println("GSM Module detected. Attempting to connect to network...");
      setLedStatus(LED_NET_STATUS_PIN, HIGH); // Temporary indicator
      // Configure GSM for text mode
      SerialGSM.println("AT+CMGF=1"); 
      delay(1000);
    } else {
      Serial.println("GSM Module response error.");
      setLedStatus(LED_NET_STATUS_PIN, LOW);
    }
  }
}

// Continuously update GPS data by feeding the incoming serial stream to the TinyGPS++ library
void updateGPS() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
  }
}

void checkSensors() {
  int smokeValue = analogRead(SMOKE_SENSOR_PIN);
  int vibrationValue = digitalRead(VIBRATION_SENSOR_PIN);
  
  Serial.printf("Smoke: %d (Threshold: %d), Vibration: %d\n", smokeValue, SMOKE_THRESHOLD, vibrationValue);

  if (smokeValue > SMOKE_THRESHOLD) {
    triggerAlert("SMOKE DETECTED");
  } else if (vibrationValue == HIGH) {
    triggerAlert("EXCESSIVE VIBRATION DETECTED (Possible Collision/Fault)");
  } else {
    // If an alert was active but conditions cleared, notify and reset
    if (isAlertActive) {
      Serial.println("Alert conditions cleared. Sending all-clear SMS.");
      String fullMessage = "ALL CLEAR: Critical safety event conditions have ceased. Location: " + 
                           String(gps.latitude(), 6) + ", " + String(gps.longitude(), 6);
      sendSMSAlert(fullMessage);
      
      isAlertActive = false;
      digitalWrite(BUZZER_PIN, LOW);
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_ALERT_PIN, LOW);
    }
  }
}

void triggerAlert(const char* reason) {
  if (!isAlertActive) {
    Serial.printf("CRITICAL ALERT TRIGGERED: %s\n", reason);
    digitalWrite(BUZZER_PIN, HIGH); // Activate buzzer
    digitalWrite(RELAY_PIN, HIGH);  // Activate relay (e.g., hazard lights)
    setLedStatus(LED_ALERT_PIN, HIGH); // Activate red alert LED
    
    // Check GPS fix before sending location
    if (gps.location.isValid() && gps.location.isUpdated()) {
      Serial.printf("Location: Lat: %.6f, Lng: %.6f\n", gps.latitude(), gps.longitude());
      String fullMessage = SMS_ALERT_TEXT;
      fullMessage += "Reason: ";
      fullMessage += reason;
      fullMessage += ". Location: ";
      fullMessage += String(gps.latitude(), 6);
      fullMessage += ", ";
      fullMessage += String(gps.longitude(), 6);
      sendSMSAlert(fullMessage);
      lastAlertTime = millis();
    } else {
      Serial.println("No valid GPS fix. Sending alert without location.");
      sendSMSAlert(SMS_ALERT_TEXT + "Reason: " + reason + ". Location: Unavailable.");
      lastAlertTime = millis();
    }
    
    isAlertActive = true;
  }
  pulseBuzzer(); // Keep pulsing the buzzer for attention
}

// Function to send SMS using AT commands
void sendSMSAlert(const String& message) {
  Serial.println("Sending SMS...");
  SerialGSM.print("AT+CMGS=\"");
  SerialGSM.print(TARGET_PHONE);
  SerialGSM.println("\"");
  delay(100);
  SerialGSM.print(message);
  SerialGSM.write(0x1A); // ASCII code for Ctrl+Z to send the message
  Serial.println("SMS command sent.");
  // Add logic to check for SMS confirmation response (optional but recommended)
}

// Helper to set LED status
void setLedStatus(int pin, bool state) {
  digitalWrite(pin, state ? HIGH : LOW);
}

// Helper function to pulse the buzzer (attention getter)
void pulseBuzzer() {
  static unsigned long lastBlinkTime = 0;
  if (isAlertActive && (millis() - lastBlinkTime > 500)) {
    digitalWrite(BUZZER_PIN, !digitalRead(BUZZER_PIN));
    lastBlinkTime = millis();
  }
}

// Update the system status LEDs for the driver
void updateSystemStatusLeds() {
  // Green LED: System Running (always on in loop)
  // Yellow LED: GSM Network Status (Blink if disconnected)
  if (isAlertActive) {
      // Red LED is handled in triggerAlert()
      // Blink the Red LED quickly to draw attention
      static unsigned long lastRedBlinkTime = 0;
      if (millis() - lastRedBlinkTime > 200) {
          digitalWrite(LED_ALERT_PIN, !digitalRead(LED_ALERT_PIN));
          lastRedBlinkTime = millis();
      }
  }

  // Simple check for network status (improve with AT+CREG for real status)
  if (!SerialGSM.available()) {
      // Blink yellow if connection is uncertain
      static unsigned long lastYellowBlinkTime = 0;
      if (millis() - lastYellowBlinkTime > 1000) {
          digitalWrite(LED_NET_STATUS_PIN, !digitalRead(LED_NET_STATUS_PIN));
          lastYellowBlinkTime = millis();
      }
  } else {
      // Yellow LED solid ON when network is confirmed (simple check)
      digitalWrite(LED_NET_STATUS_PIN, HIGH);
  }
}
