#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Preferences.h> // ESP32 Non-volatile storage
#include <WiFi.h>
#include <ESP_Mail_Client.h>

#define INTERNODE_BAUD 9600

// ============================================================
// WIFI & SMTP CONFIGURATION (UPDATE THESE)
// ============================================================
#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

#define SMTP_HOST     "smtp.gmail.com"
#define SMTP_PORT     465

#define AUTHOR_EMAIL  "your_sender_email@gmail.com"
#define AUTHOR_PASSWORD "your_email_app_password"
#define RECIPIENT_EMAIL "your_recipient_email@gmail.com"

// Hardware Pin Assignments
#define PIN_EFFLUENT_TRIG 4
#define PIN_EFFLUENT_ECHO 2
#define PIN_PH_ANALOG     13
#define PIN_TURB_ANALOG   14
#define PIN_EC_ANALOG     12
#define PIN_SUB_DS18B20   27
#define PIN_N4_DHT11      5

#define PIN_N4_UART_RX    25  // From Node 3
#define PIN_N4_UART_TX    26
#define PIN_N5_UART_RX    32  // To Controller (Mega)
#define PIN_N5_UART_TX    33

#define PIN_N4_LCD_SDA    21
#define PIN_N4_LCD_SCL    22

#define DHTTYPE           DHT11
#define ESP32_VREF        3.3f
#define ADC_RESOLUTION    4095.0f

// Tank Geometry (cm)
const float TANK_HEIGHT = 178.0f; 
const float TANK_RADIUS = 59.5f;  

// Non-Volatile Storage
Preferences preferences;

// Runtime Calibration Variables
struct CalibrationData {
    float phSlope;
    float phOffset;
    float ecKFactor;
};
static CalibrationData calData;

// Peripherals
OneWire oneWireN4(PIN_SUB_DS18B20);
DallasTemperature sensorsN4(&oneWireN4);
DHT dhtN4(PIN_N4_DHT11, DHTTYPE);
LiquidCrystal_I2C lcdN4(0x27, 16, 4);

HardwareSerial UpstreamSerial(1);
HardwareSerial DownstreamSerial(2);

struct LocalNode4Data {
    float ph;          bool phValid;
    float ntu;         bool ntuValid;
    float ec;          bool ecValid;
    float subTemp;     bool subTempValid;
    float ambTemp;     bool ambTempValid;
    float ambHum;      bool ambHumValid;
    int   evVolume;    bool evValid;
};

static LocalNode4Data g_localN4 = {0};
static SemaphoreHandle_t xLocalN4Mutex = NULL;

// SMTP Session & Message objects
SMPSession smtp;
void smtpCallback(SMTP_Status status);

void systemFatalTrap(const char* taskName) {
    Serial.printf("[FATAL ERROR] Node 4 Task Failed: %s\n", taskName);
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

void lcdSetRow(uint8_t row) {
    const uint8_t rowAddress[] = {0x00, 0x40, 0x10, 0x50};
    if (row < 4) lcdN4.command(0x80 | rowAddress[row]);
}

// Load calibration parameters
void loadCalibration() {
    preferences.begin("node4_cal", false);
    calData.phSlope   = preferences.getFloat("phSlope", 3.5f);
    calData.phOffset  = preferences.getFloat("phOffset", 0.0f);
    calData.ecKFactor = preferences.getFloat("ecKFactor", 2.0f);
    preferences.end();
}

// Initialize Wi-Fi and Synchronize NTP Time for Tunis (UTC+1)
void initNetworkAndTime() {
    Serial.printf("Connecting to Wi-Fi: %s\n", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 30) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\n[INFO] Wi-Fi Connected!");
        // Configure Tunis Time (UTC+1 = 3600 seconds offset, 0 daylight saving)
        configTime(3600, 0, "pool.ntp.org", "time.nist.gov");
        Serial.println("[INFO] Synchronizing NTP time for Tunis...");
        
        time_t nowSecs = time(nullptr);
        int retry = 0;
        while (nowSecs < 1700000000 && retry < 15) { // Ensure epoch is past valid threshold
            delay(1000);
            nowSecs = time(nullptr);
            retry++;
        }
        Serial.printf("[INFO] Current Epoch Time: %lu\n", (unsigned long)nowSecs);
    } else {
        Serial.println("\n[ERROR] Wi-Fi Connection Failed!");
    }
}

// Helper to send generic emails asynchronously/safely
bool sendMailMessage(const String& subject, const String& htmlMsg) {
    if (WiFi.status() != WL_CONNECTED) return false;

    ESP_Mail_Session session;
    session.server.host_name = SMTP_HOST;
    session.server.port = SMTP_PORT;
    session.login.email = AUTHOR_EMAIL;
    session.login.password = AUTHOR_PASSWORD;
    session.sec.type = esp_mail_secure_transport_ssl;

    SMTP_Message message;
    message.sender.name = "WattLab Node 4";
    message.sender.email = AUTHOR_EMAIL;
    message.subject = subject.c_str();
    message.addRecipient("Operator", RECIPIENT_EMAIL);
    message.html.content = htmlMsg.c_str();
    message.html.transfer_encoding = enc_base64;
    message.priority = esp_mail_priority_high;

    if (!smtp.connect(&session)) {
        Serial.printf("[SMTP ERROR] Connect failed: %s\n", smtp.errorReason().c_str());
        return false;
    }

    bool success = MailClient.sendMail(&smtp, &message);
    if (success) {
        Serial.println("[SMTP SUCCESS] Email sent successfully!");
    } else {
        Serial.printf("[SMTP ERROR] Send failed: %s\n", smtp.errorReason().c_str());
    }
    smtp.closeSession();
    return success;
}

// ============================================================
// CORE 1 TASK: SENSOR SAMPLING & CALCULATIONS (1 Hz)
// ============================================================
void Task_Sensors_Node4(void *pvParameters) {
    sensorsN4.begin();
    sensorsN4.setResolution(10);
    sensorsN4.setWaitForConversion(false);
    sensorsN4.requestTemperatures();
    dhtN4.begin();

    pinMode(PIN_EFFLUENT_TRIG, OUTPUT);
    pinMode(PIN_EFFLUENT_ECHO, INPUT);
    digitalWrite(PIN_EFFLUENT_TRIG, LOW);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    TickType_t xLastDHTTick = 0;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        TickType_t now = xTaskGetTickCount();

        LocalNode4Data localSnap;
        if (xSemaphoreTake(xLocalN4Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            localSnap = g_localN4;
            xSemaphoreGive(xLocalN4Mutex);
        }

        float currentPhSlope = calData.phSlope;
        float currentPhOffset = calData.phOffset;
        float currentEcK = calData.ecKFactor;

        // 1. pH Sensor
        int phRaw = analogRead(PIN_PH_ANALOG);
        float phVolt = (phRaw * ESP32_VREF) / ADC_RESOLUTION;
        localSnap.ph = (currentPhSlope * phVolt) + currentPhOffset;
        localSnap.phValid = (phRaw > 10 && phRaw < 4080);

        // 2. Turbidity Sensor
        int turbRaw = analogRead(PIN_TURB_ANALOG);
        float turbVolt = (turbRaw * ESP32_VREF) / ADC_RESOLUTION;
        float turbVolt5V = turbVolt * (5.0f / 3.3f); 
        if (turbVolt5V >= 4.2f) localSnap.ntu = 0.0f;
        else if (turbVolt5V < 2.5f) localSnap.ntu = 3000.0f;
        else localSnap.ntu = -1120.4f * (turbVolt5V * turbVolt5V) + 5742.3f * turbVolt5V - 4353.8f;
        if (localSnap.ntu < 0.0f) localSnap.ntu = 0.0f;
        localSnap.ntuValid = (turbRaw > 10 && turbRaw < 4080);

        // 3. Electrical Conductivity (EC)
        int ecRaw = analogRead(PIN_EC_ANALOG);
        float ecVolt = (ecRaw * ESP32_VREF) / ADC_RESOLUTION;
        localSnap.ec = ecVolt * currentEcK;
        localSnap.ecValid = (ecRaw > 10 && ecRaw < 4080);

        // 4. Submerged Temperature (DS18B20)
        float st = sensorsN4.getTempCByIndex(0);
        sensorsN4.requestTemperatures();
        if (st != DEVICE_DISCONNECTED_C && st > -55.0f && st < 125.0f) {
            localSnap.subTemp = st;
            localSnap.subTempValid = true;
        } else {
            localSnap.subTempValid = false;
        }

        // 5. Ambient Temp & Humidity (DHT11)
        if (now - xLastDHTTick >= pdMS_TO_TICKS(2000) || xLastDHTTick == 0) {
            xLastDHTTick = now;
            float at = dhtN4.readTemperature();
            float ah = dhtN4.readHumidity();
            if (!isnan(at)) { localSnap.ambTemp = at; localSnap.ambTempValid = true; } else { localSnap.ambTempValid = false; }
            if (!isnan(ah)) { localSnap.ambHum = ah; localSnap.ambHumValid = true; } else { localSnap.ambHumValid = false; }
        }

        // 6. Ultrasonic Tank Volume (HC-SR04)
        digitalWrite(PIN_EFFLUENT_TRIG, LOW);
        delayMicroseconds(2);
        digitalWrite(PIN_EFFLUENT_TRIG, HIGH);
        delayMicroseconds(10);
        digitalWrite(PIN_EFFLUENT_TRIG, LOW);

        long duration = pulseIn(PIN_EFFLUENT_ECHO, HIGH, 20000);
        if (duration > 0) {
            float distance = duration * 0.0343f / 2.0f;
            float waterHeight = TANK_HEIGHT - distance;
            if (waterHeight < 0.0f) waterHeight = 0.0f;
            if (waterHeight > TANK_HEIGHT) waterHeight = TANK_HEIGHT;
            localSnap.evVolume = (int)((3.14159f * TANK_RADIUS * TANK_RADIUS * waterHeight) / 1000.0f);
            localSnap.evValid = true;
        } else {
            localSnap.evValid = false;
        }

        if (xSemaphoreTake(xLocalN4Mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            g_localN4 = localSnap;
            xSemaphoreGive(xLocalN4Mutex);
        }
    }
}

// ============================================================
// CORE 0 TASK: BACKGROUND EMAIL, UART & CALIBRATION MANAGER
// ============================================================
void Task_Email_And_UART_Node4(void *pvParameters) {
    UpstreamSerial.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_N4_UART_RX, PIN_N4_UART_TX);
    DownstreamSerial.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_N5_UART_RX, PIN_N5_UART_TX);

    // 1. SEND STARTUP / RESET MAIL
    time_t startupTime = time(nullptr);
    char resetMsg[256];
    snprintf(resetMsg, sizeof(resetMsg), 
             "<p><b>WattLab Node 4 Alert:</b> System startup or reboot detected.</p><p>Timestamp (Tunis Time): %s</p>", ctime(&startupTime));
    sendMailMessage("[WattLab Node 4] SYSTEM RESET / STARTUP", resetMsg);

    // Persistent storage timers for rate limiting
    preferences.begin("email_state", false);
    unsigned long lastErrorEmailEpoch = preferences.getULong("lastErrEpoch", 0);
    unsigned long lastHeartbeatEpoch = preferences.getULong("lastHbEpoch", 0);
    preferences.end();

    char rxBuffer[384];
    size_t rxIdx = 0;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // Periodic Background Checks (Heartbeat every 24h, Error Checks with 12h cooldown)
        time_t currentEpoch = time(nullptr);

        // Heartbeat Check (Every 24 Hours = 86400 seconds)
        if (currentEpoch - lastHeartbeatEpoch >= 86400) {
            char hbMsg[256];
            snprintf(hbMsg, sizeof(hbMsg), 
                     "<p><b>Node 4 Status:</b> Operating normally.</p><p>Timestamp: %s</p>", ctime(&currentEpoch));
            if (sendMailMessage("[WattLab Node 4] Daily Heartbeat OK", hbMsg)) {
                lastHeartbeatEpoch = currentEpoch;
                preferences.begin("email_state", false);
                preferences.putULong("lastHbEpoch", lastHeartbeatEpoch);
                preferences.end();
            }
        }

        // Error & Environmental Alert Check (With 12-Hour Cooldown = 43200 seconds)
        LocalNode4Data checkSnap = {0};
        if (xSemaphoreTake(xLocalN4Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            checkSnap = g_localN4;
            xSemaphoreGive(xLocalN4Mutex);
        }

        bool hasError = (!checkSnap.phValid || !checkSnap.ntuValid || !checkSnap.ecValid || 
                         !checkSnap.subTempValid || !checkSnap.ambTempValid || !checkSnap.ambHumValid || !checkSnap.evValid ||
                         checkSnap.ambTemp > 45.0f || checkSnap.ambHum > 90.0f);

        if (hasError && (currentEpoch - lastErrorEmailEpoch >= 43200)) {
            String errorDetails = "<ul>";
            if (!checkSnap.phValid) errorDetails += "<li>pH Sensor Fault / Out of Range</li>";
            if (!checkSnap.ntuValid) errorDetails += "<li>Turbidity Sensor Fault</li>";
            if (!checkSnap.ecValid) errorDetails += "<li>EC Sensor Fault</li>";
            if (!checkSnap.subTempValid) errorDetails += "<li>Submerged DS18B20 Temp Sensor Disconnected/Fault</li>";
            if (!checkSnap.ambTempValid || checkSnap.ambTemp > 45.0f) errorDetails += "<li>Ambient Temperature Fault or Excessive (>45C)</li>";
            if (!checkSnap.ambHumValid || checkSnap.ambHum > 90.0f) errorDetails += "<li>Ambient Humidity Fault or Excessive (>90%)</li>";
            if (!checkSnap.evValid) errorDetails += "<li>Effluent Ultrasonic Tank Sensor Fault</li>";
            errorDetails += "</ul>";

            char errHtml[512];
            snprintf(errHtml, sizeof(errHtml), 
                     "<p><b>ATTENTION:</b> Sensor fault or excessive ambient conditions detected on Node 4!</p>%s<p>Time: %s</p>", 
                     errorDetails.c_str(), ctime(&currentEpoch));

            if (sendMailMessage("[WattLab Node 4] FAULT ALERT", errHtml)) {
                lastErrorEmailEpoch = currentEpoch;
                preferences.begin("email_state", false);
                preferences.putULong("lastErrEpoch", lastErrorEmailEpoch);
                preferences.end();
            }
        }

        // UART Forwarding Loop
        while (UpstreamSerial.available() > 0) {
            char c = (char)UpstreamSerial.read();
            if (c == '\n' || c == '\r') continue;

            if (rxIdx < sizeof(rxBuffer) - 1) {
                rxBuffer[rxIdx++] = c;
            } else {
                rxIdx = 0;
            }

            if (c == ';') {
                rxBuffer[rxIdx] = '\0';
                if (rxIdx > 0 && rxBuffer[rxIdx - 1] == ';') {
                    rxBuffer[rxIdx - 1] = '\0';
                }

                LocalNode4Data n4Snap = {0};
                if (xSemaphoreTake(xLocalN4Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                    n4Snap = g_localN4;
                    xSemaphoreGive(xLocalN4Mutex);
                }

                static char fullPacket[512];
                snprintf(fullPacket, sizeof(fullPacket),
                         "%s|pH:%.1f|Turb:%d|EC:%.1f|EV:%d|ST:%.1f|node4:1;\n",
                         rxBuffer,
                         n4Snap.phValid ? n4Snap.ph : 0.0f,
                         n4Snap.ntuValid ? (int)n4Snap.ntu : 0,
                         n4Snap.ecValid ? n4Snap.ec : 0.0f,
                         n4Snap.evValid ? n4Snap.evVolume : 0,
                         n4Snap.subTempValid ? n4Snap.subTemp : 0.0f);

                DownstreamSerial.print(fullPacket);
                rxIdx = 0;
            }
        }
    }
}

// ============================================================
// CORE 1 TASK: LOCAL LCD DISPLAY OUTPUT
// ============================================================
void Task_LCD_Node4(void *pvParameters) {
    Wire.begin(PIN_N4_LCD_SDA, PIN_N4_LCD_SCL);
    lcdN4.begin(16, 4);
    lcdN4.backlight();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        LocalNode4Data n4Snap = {0};
        if (xSemaphoreTake(xLocalN4Mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            n4Snap = g_localN4;
            xSemaphoreGive(xLocalN4Mutex);
        }

        lcdSetRow(0);
        lcdN4.printf("EV:%dL pH:%.1f   ", n4Snap.evVolume, n4Snap.ph);

        lcdSetRow(1);
        lcdN4.printf("TU:%d EC:%.1fmS   ", (int)n4Snap.ntu, n4Snap.ec);

        lcdSetRow(2);
        lcdN4.printf("ST:%.1fC AT:%.1fC  ", n4Snap.subTemp, n4Snap.ambTemp);

        lcdSetRow(3);
        lcdN4.printf("AH:%.0f%%          ", n4Snap.ambHum);
    }
}

// ============================================================
// SETUP & MAIN LOOP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    loadCalibration();

    // Initialize Wi-Fi and Synchronize Time before creating network tasks
    initNetworkAndTime();

    xLocalN4Mutex = xSemaphoreCreateMutex();
    if (!xLocalN4Mutex) systemFatalTrap("Mutex Allocation");

    BaseType_t r1 = xTaskCreatePinnedToCore(Task_Sensors_Node4,         "N4_Sensors", 4096, NULL, 2, NULL, 1);
    BaseType_t r2 = xTaskCreatePinnedToCore(Task_Email_And_UART_Node4,  "N4_EmailUART", 5120, NULL, 3, NULL, 0);
    BaseType_t r3 = xTaskCreatePinnedToCore(Task_LCD_Node4,             "N4_LCD",     3072, NULL, 1, NULL, 1);

    if (r1 != pdPASS) systemFatalTrap("N4_Sensors Task");
    if (r2 != pdPASS) systemFatalTrap("N4_EmailUART Task");
    if (r3 != pdPASS) systemFatalTrap("N4_LCD Task");
}

void loop() {
    vTaskDelete(NULL);
}