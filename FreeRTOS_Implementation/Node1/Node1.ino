#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define INTERNODE_BAUD 9600

// ============================================================
// HARDWARE PIN ASSIGNMENTS
// ============================================================
#define PIN_N1_TRIG      2
#define PIN_N1_ECHO      17

// Node 1 <-> Node 2 (Bidirectional UART)
#define PIN_N1_UART_RX   32
#define PIN_N1_UART_TX   33

#define PIN_LCD_SDA      21
#define PIN_LCD_SCL      22

// ============================================================
// SYSTEM THRESHOLDS & TANK GEOMETRY
// ============================================================
const float TANK_HEIGHT     = 260.0f;
const float TANK_RADIUS     = 110.0f;

// Environmental Safety Thresholds (Node 2 Enclosure)
const float TEMP_ALERT_TH   = 45.0f;  // Deg C (Overheat threshold)
const float HUMID_ALERT_TH  = 80.0f;  // % RH (Condensation threshold)

// Low Volume Alert Thresholds (in Liters)
const int TAV_LOW_ALERT_TH   = 5000;   // Collection Tank A Low Threshold (< 5000L)
const int TBV_LOW_ALERT_TH   = 500;    // Feeding Tank B Low Threshold (< 500L)

#define RX_TIMEOUT_MS       3000
#define TX_INTERVAL_MS      500

// ============================================================
// SHARED DATA STRUCTURES
// ============================================================
struct LocalSensorData {
    int volumeLiters;
    bool isValid;
};

struct Node2Telemetry {
    float doVal;
    float waterTemp;
    int volume;
    float ambientTemp;
    float humidity;
    bool node2Health;
    bool packetValid;
};

// ============================================================
// GLOBAL HANDLES & PERIPHERALS
// ============================================================
static LocalSensorData g_localSensors = {0, false};
static Node2Telemetry  g_node2Data    = {-1.0f, -127.0f, -1, -127.0f, -1.0f, false, false};
static uint32_t        g_lastNode2Rx = 0;

static SemaphoreHandle_t xLocalDataMutex = NULL;
static SemaphoreHandle_t xNode2DataMutex = NULL;

LiquidCrystal_I2C lcd(0x27, 16, 4);
HardwareSerial NodeSerial(1);

void systemFatalTrap(const char* taskName) {
    Serial.printf("[FATAL ERROR] System Initialization Failed: %s. Halted.\n", taskName);
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// TASK: ULTRASONIC SENSOR ACQUISITION (CORE 1)
// ============================================================
void Task_Sensors_Node1(void *pvParameters) {
    pinMode(PIN_N1_TRIG, OUTPUT);
    pinMode(PIN_N1_ECHO, INPUT);
    digitalWrite(PIN_N1_TRIG, LOW);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        digitalWrite(PIN_N1_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_N1_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_N1_TRIG, LOW);

        unsigned long duration = pulseIn(PIN_N1_ECHO, HIGH, 30000); // 30ms timeout
        
        LocalSensorData currentRead = {0, false};

        if (duration > 0) {
            // True 0-offset ultrasonic calculation
            float distance = (duration * 0.0343f / 2.0f);
            float waterHeight = TANK_HEIGHT - distance;
            if (waterHeight < 0.0f) waterHeight = 0.0f;

            float volumeLiters = (3.14159f * TANK_RADIUS * TANK_RADIUS * waterHeight) / 1000.0f;
            
            currentRead.volumeLiters = (int)(volumeLiters * 2.0f);
            currentRead.isValid = true;
        }

        if (xSemaphoreTake(xLocalDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_localSensors = currentRead;
            xSemaphoreGive(xLocalDataMutex);
        }
    }
}

// ============================================================
// FIELD PARSER FOR NODE 2 TELEMETRY
// ============================================================
String getFieldFromFrame(const String& packet, const String& fieldName) {
    String search = "|" + fieldName + ":";
    int start = packet.indexOf(search);
    if (start < 0) return "";

    start += search.length();
    int end = packet.indexOf("|", start);
    if (end < 0) end = packet.indexOf(";", start);
    if (end < 0) end = packet.length();

    return packet.substring(start, end);
}

void processNode2Packet(const String& packet) {
    String doVal      = getFieldFromFrame(packet, "DO");
    String tempVal    = getFieldFromFrame(packet, "Temp");
    String volumeVal  = getFieldFromFrame(packet, "TBV");
    String atVal      = getFieldFromFrame(packet, "AT");
    String ahVal      = getFieldFromFrame(packet, "AH");
    String n2Status   = getFieldFromFrame(packet, "node2");

    if (xSemaphoreTake(xNode2DataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (doVal.length() > 0)     g_node2Data.doVal       = doVal.toFloat();
        if (tempVal.length() > 0)   g_node2Data.waterTemp   = tempVal.toFloat();
        if (volumeVal.length() > 0) g_node2Data.volume      = volumeVal.toInt();
        if (atVal.length() > 0)     g_node2Data.ambientTemp = atVal.toFloat();
        if (ahVal.length() > 0)     g_node2Data.humidity    = ahVal.toFloat();
        if (n2Status.length() > 0)  g_node2Data.node2Health = (n2Status.toInt() == 1);

        if (doVal.length() > 0 && tempVal.length() > 0 && volumeVal.length() > 0 &&
            atVal.length() > 0 && ahVal.length() > 0 && n2Status.length() > 0) {
            g_node2Data.packetValid = true;
            g_lastNode2Rx = millis();
        }
        xSemaphoreGive(xNode2DataMutex);
    }
}

// ============================================================
// TASK: CONTINUOUS BIDIRECTIONAL UART (CORE 0)
// ============================================================
void Task_UART_Node1(void *pvParameters) {
    String rxBuffer = "";
    rxBuffer.reserve(256);

    uint32_t lastTxTime = 0;

    for (;;) {
        while (NodeSerial.available() > 0) {
            char c = (char)NodeSerial.read();

            if (c == '\n') {
                rxBuffer.trim();
                if (rxBuffer.length() > 0) {
                    processNode2Packet(rxBuffer);
                }
                rxBuffer = "";
            } else if (c != '\r') {
                rxBuffer += c;
                if (rxBuffer.length() > 250) {
                    rxBuffer = "";
                }
            }
        }

        uint32_t now = millis();
        if (now - lastTxTime >= TX_INTERVAL_MS) {
            lastTxTime = now;

            LocalSensorData localSnap = {0, false};
            if (xSemaphoreTake(xLocalDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                localSnap = g_localSensors;
                xSemaphoreGive(xLocalDataMutex);
            }

            // Standardized with leading pipe
            String outboundData = "|TAV:" + String(localSnap.volumeLiters) + 
                                  "|node1:" + String(localSnap.isValid ? 1 : 0) + ";\n";
            NodeSerial.print(outboundData);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================
// TASK: LCD RENDERING (CORE 1)
// ============================================================
void printLCDLine(byte row, const String& text) {
    lcd.setCursor(0, row);
    String line = text;
    if (line.length() > 16) {
        line = line.substring(0, 16);
    }
    lcd.print(line);
    for (int i = line.length(); i < 16; i++) {
        lcd.print(" ");
    }
}

void Task_LCD_Node1(void *pvParameters) {
    lcd.begin(16, 4);
    lcd.backlight();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        LocalSensorData localSnap = {0, false};
        Node2Telemetry node2Snap = {-1.0f, -127.0f, -1, -127.0f, -1.0f, false, false};
        uint32_t lastRx = 0;

        if (xSemaphoreTake(xLocalDataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            localSnap = g_localSensors;
            xSemaphoreGive(xLocalDataMutex);
        }
        if (xSemaphoreTake(xNode2DataMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            node2Snap = g_node2Data;
            lastRx = g_lastNode2Rx;
            xSemaphoreGive(xNode2DataMutex);
        }

        // Line 1: Tank Volumes (Strictly aligned to fit <= 16 chars)
        String line1 = "C" + String(localSnap.volumeLiters) + "L F";
        if (node2Snap.volume >= 0) line1 += String(node2Snap.volume);
        else line1 += "----";
        line1 += "L";

        // Line 2: DO (mg/l) & Water Temp (Optimized to fit exactly within 16 chars)
        String line2 = "DO";
        if (node2Snap.doVal >= 0) line2 += String(node2Snap.doVal, 1);
        else line2 += "--";
        line2.replace(".", ",");

        line2 += "mg/l T";
        if (node2Snap.waterTemp > -100.0f) line2 += String(node2Snap.waterTemp, 1);
        else line2 += "--";
        line2.replace(".", ",");
        line2 += "C";

        // Line 3: Ambient Temp & Humidity
        String line3 = "AT";
        if (node2Snap.ambientTemp > -100.0f) line3 += String(node2Snap.ambientTemp, 1);
        else line3 += "--";
        line3.replace(".", ",");

        line3 += "C H";
        if (node2Snap.humidity >= 0) line3 += String(node2Snap.humidity, 0);
        else line3 += "--";
        line3 += "%";

        // Line 4: Environmental, Volume & Communication System Status
        String statusStr = "NORMAL";

        bool highTemp  = (node2Snap.ambientTemp >= TEMP_ALERT_TH);
        bool highHumid = (node2Snap.humidity >= HUMID_ALERT_TH);

        bool lowTav    = localSnap.isValid && (localSnap.volumeLiters < TAV_LOW_ALERT_TH);
        bool lowTbv    = (node2Snap.volume >= 0) && (node2Snap.volume < TBV_LOW_ALERT_TH);

        if (lastRx == 0) {
            statusStr = "WAITING";      // Waiting for initial frame
        } else if (millis() - lastRx > RX_TIMEOUT_MS) {
            statusStr = "ERROR";         // Serial link dead
        } else if (highTemp && highHumid) {
            statusStr = "HOT+HUM";       // High temperature & high moisture
        } else if (highTemp) {
            statusStr = "OVERTEMP";      // Enclosure overheat alert
        } else if (highHumid) {
            statusStr = "HI HUMID";      // Enclosure condensation alert
        } else if (lowTav && lowTbv) {
            statusStr = "LOW C+F";       // Both Collection (TAV) and Feeding (TBV) tanks low
        } else if (lowTav) {
            statusStr = "LOW TAV";       // Collection tank (TAV < 5000L) low
        } else if (lowTbv) {
            statusStr = "LOW TBV";       // Feeding tank (TBV < 500L) low
        } else if (!node2Snap.node2Health) {
            statusStr = "FAULT";         // Hardware sensor error on Node 2
        }

        String line4 = "STATUS: " + statusStr;

        printLCDLine(0, line1);
        printLCDLine(1, line2);
        printLCDLine(2, line3);
        printLCDLine(3, line4);
    }
}

// ============================================================
// SETUP & MAIN LOOP
// ============================================================
void setup() {
    Serial.begin(115200);

    Wire.begin(PIN_LCD_SDA, PIN_LCD_SCL);
    NodeSerial.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_N1_UART_RX, PIN_N1_UART_TX);

    xLocalDataMutex = xSemaphoreCreateMutex();
    xNode2DataMutex = xSemaphoreCreateMutex();

    if (!xLocalDataMutex || !xNode2DataMutex) {
        systemFatalTrap("Mutex Allocation");
    }

    BaseType_t r1 = xTaskCreatePinnedToCore(Task_Sensors_Node1, "N1_Sensors", 3072, NULL, 2, NULL, 1);
    BaseType_t r2 = xTaskCreatePinnedToCore(Task_UART_Node1,    "N1_UART",    3072, NULL, 3, NULL, 0);
    BaseType_t r3 = xTaskCreatePinnedToCore(Task_LCD_Node1,     "N1_LCD",     3072, NULL, 1, NULL, 1);

    if (r1 != pdPASS) systemFatalTrap("N1_Sensors Task");
    if (r2 != pdPASS) systemFatalTrap("N1_UART Task");
    if (r3 != pdPASS) systemFatalTrap("N1_LCD Task");
}

void loop() {
    vTaskDelete(NULL);
}