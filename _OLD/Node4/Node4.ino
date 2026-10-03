#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ============================================================
// SERIAL SETUP
// ============================================================

HardwareSerial InSerial(2);   // RX=25, TX=26 (From Node 3)
HardwareSerial OutSerial(1);  // RX=32, TX=33 (To Next Node)

const String NODE_NAME = "node4";

unsigned long lastTxTime = 0;
const unsigned long TX_INTERVAL_MS = 500;

String incomingData = "";

// ============================================================
// PIN ALLOCATIONS
// ============================================================

// --- Ultrasonic ---
#define TRIG_PIN 4
#define ECHO_PIN 2

// --- Analog sensors ---
const int PH_PIN = 13;
const int TURBIDITY_PIN = 14;
const int EC_PIN = 12;

// --- Submerged temperature DS18B20 ---
#define DS18B20_PIN 27

// --- Ambient DHT11 ---
#define DHT_PIN 5
#define DHT_TYPE DHT11

// ============================================================
// LCD
// ============================================================

LiquidCrystal_I2C lcd(0x27, 16, 4);

// ============================================================
// TEMPERATURE SENSOR INITIALIZATION
// ============================================================

OneWire oneWire(DS18B20_PIN);
DallasTemperature submergedTempSensor(&oneWire);

DHT dht(DHT_PIN, DHT_TYPE);

// ============================================================
// TANK DIMENSIONS
// ============================================================

const float TANK_HEIGHT = 178.0;
const float TANK_RADIUS = 59.5;

// ============================================================
// ADC SETTINGS
// ============================================================

const float ESP32_VREF = 3.3;
const int ADC_RESOLUTION = 4095;

// ============================================================
// GLOBAL SENSOR VARIABLES
// ============================================================

float phValue = 0.0;
float ntu = 0.0;
float ecValue = 0.0;

float submergedTemperature = 0.0;
float ambientTemperature = 0.0;
float ambientHumidity = 0.0;

int evVolume = 0;

// ============================================================
// 16x4 LCD ROW ADDRESS FIX
// ============================================================

void lcdSetRow(uint8_t row) {
  const uint8_t rowAddress[] = {
    0x00,   // Row 1
    0x40,   // Row 2
    0x10,   // Row 3
    0x50    // Row 4
  };

  if (row > 3) {
    return;
  }

  lcd.command(0x80 | rowAddress[row]);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  InSerial.begin(9600, SERIAL_8N1, 25, 26);
  InSerial.setTimeout(30);

  OutSerial.begin(9600, SERIAL_8N1, 32, 33);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(21, 22);
  lcd.begin(16, 4);
  lcd.backlight();

  submergedTempSensor.begin();
  dht.begin();

  lcd.clear();
  lcdSetRow(0);
  lcd.print("Water Quality");
  lcdSetRow(1);
  lcd.print("Monitor Booting");

  delay(1500);
  lcd.clear();

  Serial.println("========================================");
  Serial.println("Node 4 System Initialized");
  Serial.println("16x4 LCD");
  Serial.println("DS18B20: GPIO27");
  Serial.println("DHT11: GPIO5");
  Serial.println("========================================");
}

// ============================================================
// LOOP
// ============================================================

void loop() {
  if (InSerial.available()) {
    String temp = InSerial.readStringUntil('\n');
    temp.trim();

    while (temp.length() > 0 && (temp[0] < 32 || temp[0] > 126)) {
      temp.remove(0, 1);
    }

    if (temp.length() > 0) {
      incomingData = temp;
    }
  }

  unsigned long currentMillis = millis();

  if (currentMillis - lastTxTime >= TX_INTERVAL_MS) {
    lastTxTime = currentMillis;

    // --- Read pH ---
    int phRaw = analogRead(PH_PIN);
    float phVoltage = (phRaw * ESP32_VREF) / ADC_RESOLUTION;
    phValue = 3.5 * phVoltage + 0.0;

    // --- Read Turbidity ---
    int turbRaw = analogRead(TURBIDITY_PIN);
    float turbVoltage = (turbRaw * ESP32_VREF) / ADC_RESOLUTION;

    if (turbVoltage < 2.5) {
      ntu = 3000;
    } else {
      ntu = -1120.4 * (turbVoltage * turbVoltage) + 5742.3 * turbVoltage - 4353.8;
    }
    if (ntu < 0) {
      ntu = 0;
    }

    // --- Read Electrical Conductivity ---
    int ecRaw = analogRead(EC_PIN);
    float ecVoltage = (ecRaw * ESP32_VREF) / ADC_RESOLUTION;
    ecValue = 1.0 * ecVoltage;

    // --- Read DS18B20 Submerged Temperature ---
    submergedTempSensor.requestTemperatures();
    float newSubmergedTemperature = submergedTempSensor.getTempCByIndex(0);
    if (newSubmergedTemperature != DEVICE_DISCONNECTED_C) {
      submergedTemperature = newSubmergedTemperature;
    }

    // --- Read DHT11 Ambient Temperature + Humidity ---
    float newAmbientTemperature = dht.readTemperature();
    float newAmbientHumidity = dht.readHumidity();

    if (!isnan(newAmbientTemperature)) {
      ambientTemperature = newAmbientTemperature;
    }
    if (!isnan(newAmbientHumidity)) {
      ambientHumidity = newAmbientHumidity;
    }

    // --- Calculate Effluent Tank Volume ---
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    long duration = pulseIn(ECHO_PIN, HIGH, 20000);
    float distance = duration * 0.0343 / 2.0;
    float waterHeight = TANK_HEIGHT - distance;

    if (waterHeight < 0 || duration == 0) {
      waterHeight = 0;
    }

    evVolume = (3.14159 * TANK_RADIUS * TANK_RADIUS * waterHeight) / 1000.0;

    // ========================================================
    // LCD DISPLAY
    // ========================================================

    lcd.clear();

    // Line 1
    lcdSetRow(0);
    lcd.print("EV:");
    lcd.print(evVolume);
    lcd.print("L");
    lcd.print(" pH:");
    lcd.print(phValue, 1);

    // Line 2
    lcdSetRow(1);
    lcd.print("TU:");
    lcd.print((int)ntu);
    lcd.print(" EC:");
    lcd.print(ecValue, 1);
    lcd.print("mS");

    // Line 3
    lcdSetRow(2);
    lcd.print("ST:");
    lcd.print(submergedTemperature, 1);
    lcd.print("C");
    lcd.print(" AT:");
    lcd.print(ambientTemperature, 1);
    lcd.print("C");

    // Line 4
    lcdSetRow(3);
    lcd.print("AH:");
    lcd.print((int)ambientHumidity);
    lcd.print("%");

    // ========================================================
    // BUILD PAYLOAD & TRANSMIT
    // ========================================================

    String NodeData = "pH:" + String(phValue, 1) +
                      "|Turb:" + String((int)ntu) +
                      "|EC:" + String(ecValue, 1) +
                      "|EV:" + String(evVolume) +
                      "|ST:" + String(submergedTemperature, 1) +
                      "|AT:" + String(ambientTemperature, 1) +
                      "|AH:" + String(ambientHumidity, 0) +
                      "|" + NODE_NAME + ":1;";

    String outboundData;
    if (incomingData.length() > 0) {
      String cleanIncoming = incomingData;
      if (cleanIncoming.endsWith(";")) {
        cleanIncoming.remove(cleanIncoming.length() - 1);
      }
      outboundData = cleanIncoming + "|" + NodeData;
    } else {
      outboundData = NodeData;
    }

    OutSerial.print(outboundData + "\n");
  }
}