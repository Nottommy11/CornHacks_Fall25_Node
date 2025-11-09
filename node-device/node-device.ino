/*
  ESP32 Hydroponics Hub (Text-Only Version)
  
  Reads:
  - BME280 (Air Temp, Humidity, Pressure) via I2C
  - NTC Thermistor (Water Temp) via Analog
  - TDS Sensor (Water Quality) via Analog
  - Water Level Sensor via Analog
  - Controls a 12V Grow Light via MOSFET
  
  Displays:
  - All data on a 0.91" 128x32 I2C OLED (Text-Only)
  
  Input:
  - A single button cycles through 5 display "pages".
*/

// --- Libraries ---
#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>               // For I2C
#include <math.h>               // For log() (Thermistor)
#include <Adafruit_GFX.h>       // For OLED
#include <Adafruit_SSD1306.h>   // For OLED
#include <Adafruit_Sensor.h>    // For BME280
#include <Adafruit_BME280.h>    // For BME280

// --- I2C Devices ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;

// --- Button ---
const int buttonPin = 4;
unsigned long lastButtonPress = 0;
bool lastButtonState = HIGH;
const long debounceDelay = 50;

// --- Display State Machine ---
int displayState = 0;           
const int numDisplayStates = 5; // Now 5 screens (0, 1, 2, 3, 4)
bool forceDisplayUpdate = true; 

// --- Grow Light Pin ---
// PIN CHANGED: 12 is a strapping pin. 25 is a safe, general-purpose pin.
const int LIGHT_PIN = 25; // The GPIO pin for your MOSFET Gate

// --- Shared Constants ---
#define VREF 3.3

// --- Analog Sensor Pins ---
// PIN CHANGED: 27 is ADC2 (fails with Wi-Fi). 32 is ADC1 (safe).
#define TdsSensorPin 32
const int thermistorPin = 35;
const int waterSensorPin = 34;

// --- TDS Sensor ---
#define SCOUNT  30
int analogBuffer[SCOUNT];
int analogBufferIndex = 0;
int analogBufferTemp[SCOUNT];

// --- Thermistor Sensor ---
const float referenceResistor = 10000;
const float beta = 3950;
const float nominalTemperature = 25;
const float nominalResistance = 10000;

// --- Global Sensor Variables ---
float waterTemperature = 0;
float tdsValue = 0;
float airTemperature = 0;
float airHumidity = 0;
float airPressure = 0;
String waterLevelStatus = "Init...";
int waterSensorRaw = 0;

// --- ESP-NOW Setup ---
uint8_t gatewayAddress[] = { 0x68, 0xB6, 0xB3, 0x52, 0xB3, 0x08 };

typedef struct SensorData {
  float waterTemperature;
  float airTemperature;
  float airHumidity;
  float airPressure;
  float tdsValue;
  int waterSensorRaw;
} SensorData;

SensorData dataPacket;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("[ESP-NOW] Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

//=================================================================
//   HELPER FUNCTIONS
//=================================================================

// --- Sensor Reading ---
int getMedianNum(int bArray[], int iFilterLen) {
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++) bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i]; bTab[i] = bTab[i + 1]; bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0) bTemp = bTab[(iFilterLen - 1) / 2];
  else bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}

float readWaterTemperature() {
  int adcValue = analogRead(thermistorPin);
  float voltage = (adcValue * VREF) / 4095.0;
  float resistance = (voltage * referenceResistor) / (VREF - voltage);
  float tempK = 1.0 / (((log(resistance / nominalResistance)) / beta) + (1.0 / (nominalTemperature + 273.15)));
  return tempK - 273.15;
}

void updateWaterLevel() {
  waterSensorRaw = analogRead(waterSensorPin);
  // Your calibrated values
  const int THRESH_DRY_LIMIT = 100;
  const int THRESH_MED_START = 700;
  const int THRESH_HIGH_START = 800; 

  if (waterSensorRaw < THRESH_DRY_LIMIT) {
    waterLevelStatus = "Fill Water";
  } 
  else if (waterSensorRaw >= THRESH_DRY_LIMIT && waterSensorRaw < THRESH_MED_START) {
    waterLevelStatus = "Low";
  } 
  else if (waterSensorRaw >= THRESH_MED_START && waterSensorRaw < THRESH_HIGH_START) {
    waterLevelStatus = "Medium";
  } 
  else {
    waterLevelStatus = "High";
  }
}

// --- Serial & Display ---
void printToSerial() {
  Serial.println("--- DEBUG ---");
  Serial.print("Water Temp: "); Serial.print(waterTemperature, 1); Serial.println(" C");
  Serial.print("TDS: "); Serial.print(tdsValue, 0); Serial.println(" ppm");
  Serial.print("Water Level: "); Serial.print(waterLevelStatus);
  Serial.print(" (Raw: "); Serial.print(waterSensorRaw); Serial.println(")");
  Serial.print("Air Temp: "); Serial.print(airTemperature, 1); Serial.println(" C");
  Serial.print("Humidity: "); Serial.print(airHumidity, 0); Serial.println(" %");
  Serial.print("Pressure: "); Serial.print(airPressure, 1); Serial.println(" hPa");
}

void updateOLED() {
  // *** SIMPLIFIED TEXT-ONLY VERSION ***
  
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  
  // --- Light Control Logic ---
  // The light will be OFF by default...
  digitalWrite(LIGHT_PIN, LOW);

  switch (displayState) {
    case 0: // Screen 0: Water Sensors
      display.println(F("--- WATER ---"));
      display.print(F("Temp: "));
      display.print(waterTemperature, 1);
      display.println(F(" C"));
      
      display.print(F("TDS:  "));
      display.print(tdsValue, 0);
      display.println(F(" ppm"));
      break;
      
    case 1: // Screen 1: Water Level
      display.println(F("--- WATER LEVEL ---"));
      display.setTextSize(2); // Use a bigger font
      display.setCursor(0, 16); // Center it a bit
      display.print(waterLevelStatus);
      break;
      
    case 2: // Screen 2: Air Sensors
      display.println(F("--- AIR ---"));
      display.print(F("Temp: "));
      display.print(airTemperature, 1);
      display.println(F(" C"));
      
      display.print(F("Hum:  "));
      display.print(airHumidity, 0);
      display.println(F(" %"));
      break;
      
    case 3: // Screen 3: Pressure
      display.println(F("--- PRESSURE ---"));
      display.setCursor(0, 16); // Move down a bit
      display.print(airPressure, 1);
      display.println(F(" hPa"));
      break;
      
    case 4: // Screen 4: Light Control
      display.println(F("--- GROW LIGHT ---"));
      display.setTextSize(2);
      display.setCursor(12, 16); // Centered text
      display.print(F("LIGHTS ON"));
      // ...unless it's on this screen!
      digitalWrite(LIGHT_PIN, HIGH); // Turn light ON
      break;
  }
  display.display();
}

//=================================================================
//   SETUP
//=================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Sensor Hub (Text)...");

  // --- Light Pin Setup ---
  pinMode(LIGHT_PIN, OUTPUT);
  digitalWrite(LIGHT_PIN, LOW); // Start with light OFF
  
  // --- I2C Setup ---
  Wire.begin();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Sensors Init..."));
  display.display();
  delay(1000);
  if (!bme.begin(0x76)) {
    Serial.println("BME280 not found, trying 0x77...");
    if (!bme.begin(0x77)) { Serial.println("Could not find BME280!"); }
  }

  // --- Pin Setup ---
  pinMode(TdsSensorPin, INPUT);
  pinMode(thermistorPin, INPUT);
  pinMode(waterSensorPin, INPUT);
  pinMode(buttonPin, INPUT_PULLUP);

    // --- ESP-NOW Initialization ---
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
  } else {
    Serial.println("ESP-NOW initialized.");
    esp_now_register_send_cb(OnDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, gatewayAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("Failed to add peer!");
    } else {
      Serial.println("ESP-NOW peer added successfully.");
    }
  }
}

//=================================================================
//   LOOP
//=================================================================
void loop() {
  
  // --- 1. Check for Button Press (Debounced) ---
  bool currentButtonState = (digitalRead(buttonPin) == LOW);
  if (currentButtonState != lastButtonState) {
    if (currentButtonState == true) {
      if (millis() - lastButtonPress > debounceDelay) {
        displayState = (displayState + 1) % numDisplayStates;
        forceDisplayUpdate = true;
      }
      lastButtonPress = millis();
    }
  }
  lastButtonState = currentButtonState;

  // --- 2. Non-blocking read for TDS sensor (every 40ms) ---
  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U) {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin);
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT) {
      analogBufferIndex = 0;
    }
  }

  // --- 3. Main Sensor Update (every 2 seconds OR on button press) ---
  static unsigned long sensorReadTimepoint = millis();
  if (millis() - sensorReadTimepoint > 2000U || forceDisplayUpdate) {
    
    // Read all sensors
    waterTemperature = readWaterTemperature();
    airTemperature = bme.readTemperature();
    airHumidity = bme.readHumidity();
    airPressure = bme.readPressure() / 100.0F;
    updateWaterLevel(); // Reads water level and sets status string

    // Calculate TDS
    for (int copyIndex = 0; copyIndex < SCOUNT; copyIndex++) {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    }
    float averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 4096.0;
    float compensationCoefficient = 1.0 + 0.02 * (waterTemperature - 25.0);
    float compensationVoltage = averageVoltage / compensationCoefficient;
    tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

    // --- Update all Outputs ---
    updateOLED();       // Redraw the OLED screen
    printToSerial();    // Print for debugging
    
    sensorReadTimepoint = millis();
    forceDisplayUpdate = false;

        // --- ESP-NOW Transmission ---
    dataPacket.waterTemperature = waterTemperature;
    dataPacket.airTemperature = airTemperature;
    dataPacket.airHumidity = airHumidity;
    dataPacket.airPressure = airPressure;
    dataPacket.tdsValue = tdsValue;
    dataPacket.waterSensorRaw = waterSensorRaw;

    esp_err_t result = esp_now_send(gatewayAddress, (uint8_t *)&dataPacket, sizeof(dataPacket));
    if (result == ESP_OK) {
      Serial.println("[ESP-NOW] Data sent successfully!");
    } else {
      Serial.println("[ESP-NOW] Send failed!");
    }
  }
}