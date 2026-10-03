#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ============================================================
// HARDWARE CONSTANTS & PIN DEFINITIONS
// ============================================================

#define INTERNODE_BAUD 9600

#define PIN_DS18B20 4
#define PIN_DO_ANALOG 34
#define PIN_N2_TRIG 16
#define PIN_N2_ECHO 17
#define PIN_DHT11 27
#define PIN_N1_UART_RX 25
#define PIN_N1_UART_TX 26
#define PIN_N3_UART_RX 32
#define PIN_N3_UART_TX 33

#define DHTTYPE DHT11

// DO Sensor Electrical Calibration Constants
const float VREF = 5000.0f;               // ADC Reference Voltage (mV) on 3.3V ESP32
const float ADC_RESOLUTION = 4095.0f;     // 12-bit ADC
const float TWO_POINT_VOLTAGE = 1000.0f;  // Calibration voltage threshold (mV)
const float SATURATION_DO_25C = 8.26f;    // Saturation DO concentration mg/L at 25°C

// Physical Tank Geometry (cm)
const float TANK_HEIGHT = 178.0f;
const float TANK_RADIUS = 59.5f;

// ============================================================
// HARDWARE INSTANCES
// ============================================================

OneWire oneWireN2(PIN_DS18B20);
DallasTemperature sensorsN2(&oneWireN2);
DHT dhtN2(PIN_DHT11, DHTTYPE);

HardwareSerial SerialNode1(1);
HardwareSerial SerialNode3(2);

// ============================================================
// DATA STRUCTURES
// ============================================================

struct Node2Sensors {
  float temp;
  bool tempValid;
  float doVal;
  bool doValid;
  float tbv;
  bool tbvValid;
  float at;
  bool atValid;
  float ah;
  bool ahValid;
};

struct Node1Telemetry {
  float tav;
  bool node1Status;
  bool isValid;
};

struct Node3ReverseTelemetry {
  float fr;
  bool node3Status;
  bool isValid;
};

// Global Thread-Safe Snapshots
static Node2Sensors g_n2Sensors = { 0.0f, false, 0.0f, false, 0.0f, false, 0.0f, false, 0.0f, false };
static Node1Telemetry g_n1Data = { 0.0f, false, false };
static Node3ReverseTelemetry g_n3Data = { 0.0f, false, false };

// RTOS Synchronization Primitives
static SemaphoreHandle_t xSensorsMutex = NULL;
static SemaphoreHandle_t xN1DataMutex = NULL;
static SemaphoreHandle_t xN3DataMutex = NULL;

// ============================================================
// SYSTEM ERROR TRAP
// ============================================================

void systemFatalTrap(const char* taskName) {
  Serial.printf("[FATAL ERROR] Node 2 Task Failed: %s\n", taskName);
  while (1) {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// ============================================================
// TASK: SENSOR ACQUISITION (CORE 1)
// ============================================================

void Task_Sensors_Node2(void* pvParameters) {
  sensorsN2.begin();
  sensorsN2.setWaitForConversion(false);  // Non-blocking state machine
  sensorsN2.requestTemperatures();        // Trigger initial conversion

  dhtN2.begin();

  pinMode(PIN_N2_TRIG, OUTPUT);
  pinMode(PIN_N2_ECHO, INPUT);
  digitalWrite(PIN_N2_TRIG, LOW);

  analogReadResolution(12);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // 1 Hz Periodic Loop

  TickType_t xLastDHTTick = 0;

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    TickType_t now = xTaskGetTickCount();

    Node2Sensors localSnap = { 0 };

    // Fetch current values under lock to keep prior valid state if needed
    if (xSemaphoreTake(xSensorsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      localSnap = g_n2Sensors;
      xSemaphoreGive(xSensorsMutex);
    }

    // ----------------------------------------------------
    // 1. DS18B20 Water Temperature (Non-Blocking)
    // ----------------------------------------------------
    float tempC = sensorsN2.getTempCByIndex(0);
    sensorsN2.requestTemperatures();  // Trigger conversion for NEXT cycle

    if (tempC != DEVICE_DISCONNECTED_C && tempC > -55.0f && tempC < 125.0f) {
      localSnap.temp = tempC;
      localSnap.tempValid = true;
    } else {
      localSnap.tempValid = false;
    }

    // ----------------------------------------------------
    // 2. Dissolved Oxygen (DO) + Temperature Compensation
    // ----------------------------------------------------
    int rawDO = analogRead(PIN_DO_ANALOG);
    if (rawDO > 0) {
      float doVoltage = ((float)rawDO / ADC_RESOLUTION) * VREF;  // Voltage in mV

      // Temperature compensation factor relative to 25°C
      float compTemp = localSnap.tempValid ? localSnap.temp : 25.0f;
      float compFactor = 1.0f + (compTemp - 25.0f) * (-0.02f);

      float dissolvedOxygen = (doVoltage / TWO_POINT_VOLTAGE) * SATURATION_DO_25C * compFactor;
      if (dissolvedOxygen < 0.0f) dissolvedOxygen = 0.0f;

      localSnap.doVal = dissolvedOxygen;
      localSnap.doValid = true;
    } else {
      localSnap.doValid = false;
    }

    // ----------------------------------------------------
    // 3. Ultrasonic Feeding Tank B Volume (TBV)
    // ----------------------------------------------------
    digitalWrite(PIN_N2_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_N2_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_N2_TRIG, LOW);

    unsigned long duration = pulseIn(PIN_N2_ECHO, HIGH, 20000);  // 20ms timeout (~3.4m)
    if (duration > 0) {
      // Distance in cm
      float distanceCm = (duration * 0.0343f) / 2.0f;

      // Water height in cm (TANK_HEIGHT = 178.0 cm)
      float waterHeight = TANK_HEIGHT - distanceCm;
      if (waterHeight < 0.0f) waterHeight = 0.0f;
      if (waterHeight > TANK_HEIGHT) waterHeight = TANK_HEIGHT;

      // Cylinder Volume in Liters: (pi * r^2 * h) / 1000
      // TANK_RADIUS = 59.5 cm
      float volumeLiters = (3.14159f * TANK_RADIUS * TANK_RADIUS * waterHeight) / 1000.0f;

      localSnap.tbv = volumeLiters;
      localSnap.tbvValid = true;
    } else {
      localSnap.tbvValid = false;
    }

    // ----------------------------------------------------
    // 4. DHT11 Ambient Sensors (Paced Every >= 2000 ms)
    // ----------------------------------------------------
    if (now - xLastDHTTick >= pdMS_TO_TICKS(2000) || xLastDHTTick == 0) {
      xLastDHTTick = now;
      float tVal = dhtN2.readTemperature();
      float hVal = dhtN2.readHumidity();

      if (!isnan(tVal)) {
        localSnap.at = tVal;
        localSnap.atValid = true;
      } else {
        localSnap.atValid = false;
      }

      if (!isnan(hVal)) {
        localSnap.ah = hVal;
        localSnap.ahValid = true;
      } else {
        localSnap.ahValid = false;
      }
    }

    // Commit updated state to global thread-safe store
    if (xSemaphoreTake(xSensorsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
      g_n2Sensors = localSnap;
      xSemaphoreGive(xSensorsMutex);
    }
  }
}

// ============================================================
// PACKET PARSERS
// ============================================================

bool parseNode1Packet(const char* frame, Node1Telemetry& outN1) {
  if (!frame) return false;
  const char* pTAV = strstr(frame, "|TAV:");
  const char* pNode1 = strstr(frame, "|node1:");
  if (!pTAV || !pNode1) return false;

  int n1Status = 0;
  if (sscanf(pTAV, "|TAV:%f", &outN1.tav) != 1) return false;
  if (sscanf(pNode1, "|node1:%d", &n1Status) != 1) return false;

  outN1.node1Status = (n1Status == 1);
  outN1.isValid = true;
  return true;
}

bool parseNode3ReversePacket(const char* frame, Node3ReverseTelemetry& outN3) {
  if (!frame) return false;
  const char* pFR = strstr(frame, "|FR:");
  const char* pNode3 = strstr(frame, "|node3:");
  if (!pFR || !pNode3) return false;

  int n3Status = 0;
  if (sscanf(pFR, "|FR:%f", &outN3.fr) != 1) return false;
  if (sscanf(pNode3, "|node3:%d", &n3Status) != 1) return false;

  outN3.node3Status = (n3Status == 1);
  outN3.isValid = true;
  return true;
}

// ============================================================
// TASK: INTER-NODE UART ROUTING & MULTIPLEXING (CORE 0)
// ============================================================

void Task_UART_Node2(void* pvParameters) {
  SerialNode1.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_N1_UART_RX, PIN_N1_UART_TX);
  SerialNode3.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_N3_UART_RX, PIN_N3_UART_TX);

  char rxBufferN1[128], rxBufferN3[128];
  size_t rxIdxN1 = 0, rxIdxN3 = 0;

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50);  // 20 Hz Polling

  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // ----------------------------------------------------
    // 1. Read Upstream Telemetry from Node 1
    // ----------------------------------------------------
    while (SerialNode1.available() > 0) {
      char c = (char)SerialNode1.read();
      if (c == '\n' || c == '\r') continue;
      if (rxIdxN1 < sizeof(rxBufferN1) - 1) {
        rxBufferN1[rxIdxN1++] = c;
      } else {
        rxIdxN1 = 0;  // Buffer overflow safety reset
      }

      if (c == ';') {
        rxBufferN1[rxIdxN1] = '\0';
        Node1Telemetry parsedN1 = { 0 };
        if (parseNode1Packet(rxBufferN1, parsedN1)) {
          if (xSemaphoreTake(xN1DataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_n1Data = parsedN1;
            xSemaphoreGive(xN1DataMutex);
          }
        }
        rxIdxN1 = 0;
      }
    }

    // ----------------------------------------------------
    // 2. Read Reverse Telemetry from Node 3
    // ----------------------------------------------------
    while (SerialNode3.available() > 0) {
      char c = (char)SerialNode3.read();
      if (c == '\n' || c == '\r') continue;
      if (rxIdxN3 < sizeof(rxBufferN3) - 1) {
        rxBufferN3[rxIdxN3++] = c;
      } else {
        rxIdxN3 = 0;  // Buffer overflow safety reset
      }

      if (c == ';') {
        rxBufferN3[rxIdxN3] = '\0';
        Node3ReverseTelemetry parsedN3 = { 0 };
        if (parseNode3ReversePacket(rxBufferN3, parsedN3)) {
          if (xSemaphoreTake(xN3DataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_n3Data = parsedN3;  // Fixed variable target
            xSemaphoreGive(xN3DataMutex);
          }
        }
        rxIdxN3 = 0;
      }
    }

    // ----------------------------------------------------
    // 3. Construct Consolidated Payload & Send Downstream to Node 3
    // ----------------------------------------------------
    Node2Sensors sSnap = { 0 };
    Node1Telemetry n1Snap = { 0 };

    if (xSemaphoreTake(xSensorsMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      sSnap = g_n2Sensors;
      xSemaphoreGive(xSensorsMutex);
    }
    if (xSemaphoreTake(xN1DataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      n1Snap = g_n1Data;
      xSemaphoreGive(xN1DataMutex);
    }

    // Compute overall Node 2 status dynamically based on validity flags
    bool node2Healthy = sSnap.tempValid && sSnap.doValid && sSnap.tbvValid && sSnap.atValid && sSnap.ahValid;

    char downstreamPayload[256];
    snprintf(downstreamPayload, sizeof(downstreamPayload),
             "|TAV:%.2f|node1:%d|DO:%.2f|Temp:%.2f|TBV:%.2f|AT:%.2f|AH:%.2f|node2:%d;\n",
             n1Snap.isValid ? n1Snap.tav : 0.0f,
             (n1Snap.isValid && n1Snap.node1Status) ? 1 : 0,
             sSnap.doValid ? sSnap.doVal : 0.0f,
             sSnap.tempValid ? sSnap.temp : 0.0f,
             sSnap.tbvValid ? sSnap.tbv : 0.0f,
             sSnap.atValid ? sSnap.at : 0.0f,
             sSnap.ahValid ? sSnap.ah : 0.0f,
             node2Healthy ? 1 : 0);

    SerialNode3.print(downstreamPayload);

    // ----------------------------------------------------
    // 4. Echo Upstream Telemetry back to Node 1 for Local Display
    // ----------------------------------------------------
    char echoToN1[256];
    snprintf(echoToN1, sizeof(echoToN1),
             "|DO:%.2f|Temp:%.2f|TBV:%.2f|AT:%.2f|AH:%.2f|node2:%d;\n",
             sSnap.doValid ? sSnap.doVal : 0.0f,
             sSnap.tempValid ? sSnap.temp : 0.0f,
             sSnap.tbvValid ? sSnap.tbv : 0.0f,
             sSnap.atValid ? sSnap.at : 0.0f,
             sSnap.ahValid ? sSnap.ah : 0.0f,
             node2Healthy ? 1 : 0);

    SerialNode1.print(echoToN1);
  }
}

// ============================================================
// INITIALIZATION
// ============================================================

void setup() {
  Serial.begin(115200);

  // Create RTOS Mutexes
  xSensorsMutex = xSemaphoreCreateMutex();
  xN1DataMutex = xSemaphoreCreateMutex();
  xN3DataMutex = xSemaphoreCreateMutex();

  if (!xSensorsMutex || !xN1DataMutex || !xN3DataMutex) {
    systemFatalTrap("Mutex Allocation Failure");
  }

  // Pin Sensor Task to Core 1, UART Communication Task to Core 0
  BaseType_t r1 = xTaskCreatePinnedToCore(Task_Sensors_Node2, "N2_Sensors", 4096, NULL, 2, NULL, 1);
  BaseType_t r2 = xTaskCreatePinnedToCore(Task_UART_Node2, "N2_UART", 4096, NULL, 3, NULL, 0);

  if (r1 != pdPASS) systemFatalTrap("Task N2_Sensors Creation");
  if (r2 != pdPASS) systemFatalTrap("Task N2_UART Creation");
}

void loop() {
  // Delete setup thread to reclaim memory; RTOS tasks run independently
  vTaskDelete(NULL);
}