
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <Preferences.h> // ESP32 Non-volatile storage for calibration

#define INTERNODE_BAUD 9600

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

// IMPORTANT:
// ESP32 ADC is NOT a true 0-5V ADC.
// The sensor boards may be powered from 5V, but their analog
// output must remain within the ESP32 ADC input range.
//
// ADC attenuation is configured below.
#define ADC_RESOLUTION    4095.0f

// Tank Geometry (cm)
const float TANK_HEIGHT = 178.0f;
const float TANK_RADIUS = 59.5f;

// ============================================================
// ADC CONFIGURATION
// ============================================================

// Number of samples used for analog filtering
#define ADC_SAMPLES 16

// ESP32 ADC attenuation.
// 11 dB gives the largest practical input range.
#define SENSOR_ADC_ATTENUATION ADC_11db

// ============================================================
// NON-VOLATILE STORAGE
// ============================================================

Preferences preferences;

struct CalibrationData {
    float phSlope;
    float phOffset;
    float ecKFactor;
};

static CalibrationData calData;

// ============================================================
// PERIPHERALS
// ============================================================

OneWire oneWireN4(PIN_SUB_DS18B20);
DallasTemperature sensorsN4(&oneWireN4);

DHT dhtN4(PIN_N4_DHT11, DHTTYPE);

LiquidCrystal_I2C lcdN4(0x27, 16, 4);

HardwareSerial UpstreamSerial(1);
HardwareSerial DownstreamSerial(2);

// ============================================================
// LOCAL DATA
// ============================================================

struct LocalNode4Data {
    float ph;       bool phValid;
    float ntu;      bool ntuValid;
    float ec;       bool ecValid;
    float subTemp;  bool subTempValid;
    float ambTemp;  bool ambTempValid;
    float ambHum;   bool ambHumValid;
    int   tcVolume; bool tcValid;
};

static LocalNode4Data g_localN4 = {0};

static SemaphoreHandle_t xLocalN4Mutex = NULL;

// ============================================================
// FATAL ERROR
// ============================================================

void systemFatalTrap(const char* taskName) {
    Serial.printf("[FATAL ERROR] Node 4 Task Failed: %s\n", taskName);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================
// LCD ROW HELPER
// ============================================================

void lcdSetRow(uint8_t row) {
    const uint8_t rowAddress[] = {
        0x00,
        0x40,
        0x10,
        0x50
    };

    if (row < 4) {
        lcdN4.command(0x80 | rowAddress[row]);
    }
}

// ============================================================
// LOAD CALIBRATION
// ============================================================

void loadCalibration() {

    preferences.begin("node4_cal", false);

    calData.phSlope =
        preferences.getFloat("phSlope", 3.5f);

    calData.phOffset =
        preferences.getFloat("phOffset", -1.75f);

    calData.ecKFactor =
        preferences.getFloat("ecKFactor", 9.997f);

    preferences.end();

    Serial.println("--- Loaded Calibration Constants ---");

    Serial.printf(
        "pH Slope: %.3f | pH Offset: %.3f | EC K-Factor: %.3f\n",
        calData.phSlope,
        calData.phOffset,
        calData.ecKFactor
    );
}

// ============================================================
// ADC INITIALIZATION
// ============================================================

void configureADC() {

    pinMode(PIN_PH_ANALOG, INPUT);
    pinMode(PIN_TURB_ANALOG, INPUT);
    pinMode(PIN_EC_ANALOG, INPUT);

    analogSetPinAttenuation(
        PIN_PH_ANALOG,
        SENSOR_ADC_ATTENUATION
    );

    analogSetPinAttenuation(
        PIN_TURB_ANALOG,
        SENSOR_ADC_ATTENUATION
    );

    analogSetPinAttenuation(
        PIN_EC_ANALOG,
        SENSOR_ADC_ATTENUATION
    );

    // 12-bit ADC
    analogReadResolution(12);

    Serial.println("[ADC] Resolution: 12-bit");
    Serial.println("[ADC] Attenuation: 11 dB");
    Serial.println("[ADC] Multi-sample filtering enabled");
}

// ============================================================
// FILTERED ADC READ
// ============================================================

uint16_t readADCFiltered(uint8_t pin) {

    uint32_t total = 0;

    // Throw away the first reading after switching channels.
    // This helps reduce channel-to-channel ADC residue.
    analogRead(pin);

    delayMicroseconds(100);

    for (int i = 0; i < ADC_SAMPLES; i++) {

        total += analogRead(pin);

        delayMicroseconds(150);
    }

    return (uint16_t)(total / ADC_SAMPLES);
}

// ============================================================
// ADC VOLTAGE
// ============================================================

float adcToVoltage(uint16_t raw) {

    /*
       ESP32 ADC readings are affected by attenuation and ADC
       characteristics. analogReadMilliVolts() performs the
       ESP32 calibration-aware conversion where supported.

       We use the averaged raw value and convert it using the
       calibrated millivolt reading path.
    */

    uint32_t totalMv = 0;

    for (int i = 0; i < 4; i++) {
        totalMv += analogReadMilliVolts(PIN_PH_ANALOG);
        delayMicroseconds(100);
    }

    // This helper is only used for diagnostics if called directly.
    // The sensor-specific functions below perform their own
    // calibrated voltage acquisition.
    return (raw / ADC_RESOLUTION) * 3.3f;
}

// ============================================================
// CALIBRATED AVERAGE VOLTAGE
// ============================================================

float readSensorVoltage(uint8_t pin) {

    uint32_t totalMv = 0;

    // Discard first sample after ADC channel switching.
    analogRead(pin);
    delayMicroseconds(100);

    for (int i = 0; i < ADC_SAMPLES; i++) {

        totalMv += analogReadMilliVolts(pin);

        delayMicroseconds(150);
    }

    float voltage =
        ((float)totalMv / ADC_SAMPLES) / 1000.0f;

    return voltage;
}

// ============================================================
// DS18B20 INITIALIZATION
// ============================================================

void initializeDS18B20() {

    sensorsN4.begin();

    sensorsN4.setResolution(10);

    /*
       We intentionally use blocking conversion here.

       At 10-bit resolution the conversion takes approximately
       187.5 ms, which is completely acceptable because the
       sensor task runs at 1 Hz.
    */
    sensorsN4.setWaitForConversion(true);

    Serial.printf(
        "[DS18B20] Devices found: %d\n",
        sensorsN4.getDeviceCount()
    );

    if (sensorsN4.getDeviceCount() == 0) {
        Serial.println(
            "[DS18B20] WARNING: No temperature sensor detected!"
        );
    }
}

// ============================================================
// DHT INITIALIZATION
// ============================================================

void initializeDHT() {

    dhtN4.begin();

    Serial.println("[DHT11] Initialized");
}

// ============================================================
// ULTRASONIC READING
// ============================================================

bool readTankVolume(int &volumeLiters) {

    digitalWrite(PIN_EFFLUENT_TRIG, LOW);
    delayMicroseconds(3);

    digitalWrite(PIN_EFFLUENT_TRIG, HIGH);
    delayMicroseconds(10);

    digitalWrite(PIN_EFFLUENT_TRIG, LOW);

    long duration =
        pulseIn(
            PIN_EFFLUENT_ECHO,
            HIGH,
            25000
        );

    if (duration <= 0) {
        return false;
    }

    float distance =
        duration * 0.0343f / 2.0f;

    // Reject physically impossible measurements
    if (distance < 0.0f ||
        distance > TANK_HEIGHT + 20.0f) {
        return false;
    }

    float waterHeight =
        TANK_HEIGHT - distance;

    if (waterHeight < 0.0f) {
        waterHeight = 0.0f;
    }

    if (waterHeight > TANK_HEIGHT) {
        waterHeight = TANK_HEIGHT;
    }

    float volume =
        (3.14159f *
         TANK_RADIUS *
         TANK_RADIUS *
         waterHeight) / 1000.0f;

    if (volume < 0.0f) {
        volume = 0.0f;
    }

    volumeLiters = (int)volume;

    return true;
}

// ============================================================
// CORE 1 TASK:
// SENSOR SAMPLING & CALCULATIONS
// ============================================================

void Task_Sensors_Node4(void *pvParameters) {

    configureADC();

    initializeDS18B20();

    initializeDHT();

    pinMode(PIN_EFFLUENT_TRIG, OUTPUT);
    pinMode(PIN_EFFLUENT_ECHO, INPUT);

    digitalWrite(
        PIN_EFFLUENT_TRIG,
        LOW
    );

    TickType_t xLastWakeTime =
        xTaskGetTickCount();

    const TickType_t xFrequency =
        pdMS_TO_TICKS(1000);

    TickType_t xLastDHTTick = 0;

    for (;;) {

        vTaskDelayUntil(
            &xLastWakeTime,
            xFrequency
        );

        TickType_t now =
            xTaskGetTickCount();

        LocalNode4Data localSnap;

        // ----------------------------------------------------
        // COPY CURRENT DATA
        // ----------------------------------------------------

        if (xSemaphoreTake(
                xLocalN4Mutex,
                pdMS_TO_TICKS(50)
            ) == pdTRUE) {

            localSnap = g_localN4;

            xSemaphoreGive(
                xLocalN4Mutex
            );
        }

        // ====================================================
        // 1. DS18B20 SUBMERGED TEMPERATURE
        // ====================================================

        float st = DEVICE_DISCONNECTED_C;

        if (sensorsN4.getDeviceCount() > 0) {

            sensorsN4.requestTemperatures();

            st =
                sensorsN4.getTempCByIndex(0);
        }

        if (st != DEVICE_DISCONNECTED_C &&
            st >= -55.0f &&
            st <= 125.0f) {

            localSnap.subTemp = st;
            localSnap.subTempValid = true;

        } else {

            localSnap.subTempValid = false;
        }

        float validTempForComp =
            localSnap.subTempValid
            ? localSnap.subTemp
            : 25.0f;

        // ====================================================
        // 2. pH SENSOR
        // ====================================================

        uint16_t phRaw =
            readADCFiltered(PIN_PH_ANALOG);

        float phVolt =
            readSensorVoltage(PIN_PH_ANALOG);

        /*
           Existing calibration model preserved:

               pHcalculated =
                   slope * voltage + offset

           Temperature compensation also preserved.
        */

        float phCompensationCoefficient =
            1.0f +
            0.02f *
            (validTempForComp - 25.0f);

        float calculatedPh =
            (calData.phSlope * phVolt) +
            calData.phOffset;

        float finalPh =
            7.0f +
            ((calculatedPh - 7.0f) /
             phCompensationCoefficient);

        /*
           Basic sanity check.

           A completely saturated / disconnected ADC signal
           should not be reported as a valid pH measurement.
        */

        if (phVolt >= 0.02f &&
            phVolt <= 3.30f &&
            finalPh >= 0.0f &&
            finalPh <= 14.0f) {

            localSnap.ph = finalPh;
            localSnap.phValid = true;

        } else {

            localSnap.phValid = false;
        }

        // ====================================================
        // 3. TURBIDITY
        // ====================================================

        uint16_t turbRaw =
            readADCFiltered(PIN_TURB_ANALOG);

        float turbVolt =
            readSensorVoltage(PIN_TURB_ANALOG);

        float ntu = 0.0f;

        /*
           SEN0189 / DFRobot turbidity relationship.

           IMPORTANT:
           The original code used 5.0 V as the ADC reference.
           That is incorrect for the ESP32 ADC measurement.

           The voltage here is the actual voltage measured at
           the ESP32 ADC input.
        */

        if (turbVolt >= 3.20f) {

            // Very clear water / sensor output near upper range
            ntu = 0.0f;

        }
        else if (turbVolt <= 0.50f) {

            // Extremely turbid / below useful formula range
            ntu = 3000.0f;

        }
        else {

            ntu =
                -1120.4f *
                (turbVolt * turbVolt)

                + 5742.3f *
                turbVolt

                - 4353.8f;

            if (ntu < 0.0f) {
                ntu = 0.0f;
            }

            if (ntu > 3000.0f) {
                ntu = 3000.0f;
            }
        }

        /*
           Don't automatically invalidate the turbidity sensor
           just because it is reading near one end of its range.

           Saturation is a valid physical condition.
        */

        if (turbVolt >= 0.0f &&
            turbVolt <= 3.30f) {

            localSnap.ntu = ntu;
            localSnap.ntuValid = true;

        } else {

            localSnap.ntuValid = false;
        }

        // ====================================================
        // 4. EC SENSOR
        // ====================================================

        uint16_t ecRaw =
            readADCFiltered(PIN_EC_ANALOG);

        float ecVolt =
            readSensorVoltage(PIN_EC_ANALOG);

        /*
           Existing K-factor behavior preserved.

           Original:
               EC(mS/cm) = voltage * K

           Then temperature compensation and conversion to uS/cm.
        */

        float ecCoefficient =
            1.0f +
            0.0185f *
            (validTempForComp - 25.0f);

        float rawEcMilliSiemens =
            ecVolt * calData.ecKFactor;

        float compensatedEc =
            (rawEcMilliSiemens /
             ecCoefficient) *
            1000.0f;

        if (compensatedEc < 0.0f) {
            compensatedEc = 0.0f;
        }

        /*
           Reject impossible ADC conditions but do not reject
           a zero EC value — zero/very-low conductivity is a
           legitimate measurement.
        */

        if (ecVolt >= 0.0f &&
            ecVolt <= 3.30f) {

            localSnap.ec = compensatedEc;
            localSnap.ecValid = true;

        } else {

            localSnap.ecValid = false;
        }

        // ====================================================
        // 5. DHT11
        // ====================================================

        if (
            xLastDHTTick == 0 ||
            now - xLastDHTTick >=
            pdMS_TO_TICKS(2000)
        ) {

            xLastDHTTick = now;

            float at =
                dhtN4.readTemperature();

            float ah =
                dhtN4.readHumidity();

            if (!isnan(at) &&
                at >= -20.0f &&
                at <= 80.0f) {

                localSnap.ambTemp = at;
                localSnap.ambTempValid = true;

            } else {

                localSnap.ambTempValid = false;
            }

            if (!isnan(ah) &&
                ah >= 0.0f &&
                ah <= 100.0f) {

                localSnap.ambHum = ah;
                localSnap.ambHumValid = true;

            } else {

                localSnap.ambHumValid = false;
            }
        }

        // ====================================================
        // 6. HC-SR04 TANK VOLUME
        // ====================================================

        int tankVolume = 0;

        if (readTankVolume(tankVolume)) {

            localSnap.tcVolume = tankVolume;
            localSnap.tcValid = true;

        } else {

            localSnap.tcValid = false;
        }

        // ====================================================
        // STORE SENSOR SNAPSHOT
        // ====================================================

        if (xSemaphoreTake(
                xLocalN4Mutex,
                pdMS_TO_TICKS(50)
            ) == pdTRUE) {

            g_localN4 = localSnap;

            xSemaphoreGive(
                xLocalN4Mutex
            );
        }
    }
}

// ============================================================
// PC SERIAL DEBUG & CALIBRATION CONSOLE
// ============================================================

void handleSerialCalibrationCommands() {

    if (Serial.available() > 0) {

        String cmd =
            Serial.readStringUntil('\n');

        cmd.trim();

        if (cmd.equalsIgnoreCase("HELP")) {

            Serial.println(
                "\n========== NODE 4 CALIBRATION DEBUG CONSOLE =========="
            );

            Serial.println(
                "STATUS        - View current raw ADC and calculated values"
            );

            Serial.println(
                "SET:EC_K=<val>- Set and save EC K factor"
            );

            Serial.println(
                "SET:PH_S=<val>- Set and save pH slope"
            );

            Serial.println(
                "SET:PH_O=<val>- Set and save pH offset"
            );

            Serial.println(
                "RESET         - Reset calibration to factory defaults"
            );

            Serial.println(
                "=====================================================\n"
            );
        }

        // ====================================================
        // STATUS
        // ====================================================

        else if (cmd.equalsIgnoreCase("STATUS")) {

            uint16_t phRaw =
                readADCFiltered(PIN_PH_ANALOG);

            uint16_t ecRaw =
                readADCFiltered(PIN_EC_ANALOG);

            uint16_t turbRaw =
                readADCFiltered(PIN_TURB_ANALOG);

            float phVoltage =
                readSensorVoltage(PIN_PH_ANALOG);

            float ecVoltage =
                readSensorVoltage(PIN_EC_ANALOG);

            float turbVoltage =
                readSensorVoltage(PIN_TURB_ANALOG);

            Serial.println(
                "\n--- Live Raw Sensor Diagnostics ---"
            );

            Serial.printf(
                "pH   -> Raw ADC: %4d | Voltage: %.3fV\n",
                phRaw,
                phVoltage
            );

            Serial.printf(
                "EC   -> Raw ADC: %4d | Voltage: %.3fV | K: %.3f\n",
                ecRaw,
                ecVoltage,
                calData.ecKFactor
            );

            Serial.printf(
                "Turb -> Raw ADC: %4d | Voltage: %.3fV\n",
                turbRaw,
                turbVoltage
            );

            Serial.printf(
                "Saved Constants -> pH Slope: %.3f | pH Offset: %.3f | EC K: %.3f\n\n",
                calData.phSlope,
                calData.phOffset,
                calData.ecKFactor
            );
        }

        // ====================================================
        // EC K
        // ====================================================

        else if (cmd.startsWith("SET:EC_K=")) {

            float val =
                cmd.substring(9).toFloat();

            if (val > 0.0f) {

                calData.ecKFactor = val;

                preferences.begin(
                    "node4_cal",
                    false
                );

                preferences.putFloat(
                    "ecKFactor",
                    calData.ecKFactor
                );

                preferences.end();

                Serial.printf(
                    "[SUCCESS] EC K-Factor updated and saved to flash: %.3f\n",
                    calData.ecKFactor
                );

            } else {

                Serial.println(
                    "[ERROR] Invalid value for EC K-Factor."
                );
            }
        }

        // ====================================================
        // pH SLOPE
        // ====================================================

        else if (cmd.startsWith("SET:PH_S=")) {

            float val =
                cmd.substring(9).toFloat();

            calData.phSlope = val;

            preferences.begin(
                "node4_cal",
                false
            );

            preferences.putFloat(
                "phSlope",
                calData.phSlope
            );

            preferences.end();

            Serial.printf(
                "[SUCCESS] pH Slope updated and saved: %.3f\n",
                calData.phSlope
            );
        }

        // ====================================================
        // pH OFFSET
        // ====================================================

        else if (cmd.startsWith("SET:PH_O=")) {

            float val =
                cmd.substring(9).toFloat();

            calData.phOffset = val;

            preferences.begin(
                "node4_cal",
                false
            );

            preferences.putFloat(
                "phOffset",
                calData.phOffset
            );

            preferences.end();

            Serial.printf(
                "[SUCCESS] pH Offset updated and saved: %.3f\n",
                calData.phOffset
            );
        }

        // ====================================================
        // RESET
        // ====================================================

        else if (cmd.equalsIgnoreCase("RESET")) {

            preferences.begin(
                "node4_cal",
                false
            );

            preferences.clear();

            preferences.end();

            loadCalibration();

            Serial.println(
                "[SUCCESS] Calibration reset to defaults."
            );
        }
    }
}

// ============================================================
// CLEAN UPSTREAM PACKET
// ============================================================

String cleanUpstreamPacket(String rawPacket) {

    String cleaned = "";

    int startIdx = 0;

    while (true) {

        int pipeIdx =
            rawPacket.indexOf(
                '|',
                startIdx
            );

        String token = "";

        if (pipeIdx == -1) {

            token =
                rawPacket.substring(
                    startIdx
                );

        } else {

            token =
                rawPacket.substring(
                    startIdx,
                    pipeIdx
                );
        }

        if (
            !token.startsWith("AT:") &&
            !token.startsWith("AH:")
        ) {

            if (cleaned.length() > 0) {
                cleaned += "|";
            }

            cleaned += token;
        }

        if (pipeIdx == -1) {
            break;
        }

        startIdx =
            pipeIdx + 1;
    }

    return cleaned;
}

// ============================================================
// CORE 0 TASK:
// DUAL UART FORWARDING & PACKET CLEANING
// ============================================================

void Task_UART_Node4(void *pvParameters) {

    UpstreamSerial.begin(
        INTERNODE_BAUD,
        SERIAL_8N1,
        PIN_N4_UART_RX,
        PIN_N4_UART_TX
    );

    DownstreamSerial.begin(
        INTERNODE_BAUD,
        SERIAL_8N1,
        PIN_N5_UART_RX,
        PIN_N5_UART_TX
    );

    char rxBuffer[384];

    size_t rxIdx = 0;

    TickType_t xLastWakeTime =
        xTaskGetTickCount();

    const TickType_t xFrequency =
        pdMS_TO_TICKS(50);

    for (;;) {

        vTaskDelayUntil(
            &xLastWakeTime,
            xFrequency
        );

        handleSerialCalibrationCommands();

        while (UpstreamSerial.available() > 0) {

            char c =
                (char)UpstreamSerial.read();

            if (c == '\n' ||
                c == '\r') {

                continue;
            }

            if (rxIdx <
                sizeof(rxBuffer) - 1) {

                rxBuffer[rxIdx++] = c;

            } else {

                rxIdx = 0;
            }

            if (c == ';') {

                rxBuffer[rxIdx] = '\0';

                if (
                    rxIdx > 0 &&
                    rxBuffer[rxIdx - 1] == ';'
                ) {

                    rxBuffer[rxIdx - 1] =
                        '\0';
                }

                LocalNode4Data n4Snap = {0};

                if (
                    xSemaphoreTake(
                        xLocalN4Mutex,
                        pdMS_TO_TICKS(20)
                    ) == pdTRUE
                ) {

                    n4Snap = g_localN4;

                    xSemaphoreGive(
                        xLocalN4Mutex
                    );
                }

                String cleanedUpstream =
                    cleanUpstreamPacket(
                        String(rxBuffer)
                    );

                Serial.println(
                    "\n================ [NODE 4 PACKET TRANSACTION] ================"
                );

                Serial.printf(
                    "[INCOMING] Raw Packet from Upstream: %s;\n",
                    rxBuffer
                );

                Serial.printf(
                    "[CLEANED] Upstream Packet: %s\n",
                    cleanedUpstream.c_str()
                );

                Serial.println(
                    "[LOCAL SENSORS] Current Node 4 Readings:"
                );

                Serial.printf(
                    "  - pH:       %.1f (Valid: %s)\n",
                    n4Snap.ph,
                    n4Snap.phValid
                        ? "YES"
                        : "NO"
                );

                Serial.printf(
                    "  - Turbidity:%.1f NTU (Valid: %s)\n",
                    n4Snap.ntu,
                    n4Snap.ntuValid
                        ? "YES"
                        : "NO"
                );

                Serial.printf(
                    "  - EC:       %.0f uS/cm (Valid: %s)\n",
                    n4Snap.ec,
                    n4Snap.ecValid
                        ? "YES"
                        : "NO"
                );

                Serial.printf(
                    "  - TCV Volume:%d L (Valid: %s)\n",
                    n4Snap.tcVolume,
                    n4Snap.tcValid
                        ? "YES"
                        : "NO"
                );

                static char fullPacket[512];

                snprintf(
                    fullPacket,
                    sizeof(fullPacket),

                    "%s|pH:%.1f|Turb:%d|EC:%.0f|TCV:%d|node4:1;\n",

                    cleanedUpstream.c_str(),

                    n4Snap.phValid
                        ? n4Snap.ph
                        : 0.0f,

                    n4Snap.ntuValid
                        ? (int)n4Snap.ntu
                        : 0,

                    n4Snap.ecValid
                        ? n4Snap.ec
                        : 0.0f,

                    n4Snap.tcValid
                        ? n4Snap.tcVolume
                        : 0
                );

                Serial.printf(
                    "[OUTGOING] Final Packet Sent Downstream: %s",
                    fullPacket
                );

                Serial.println(
                    "=============================================================\n"
                );

                DownstreamSerial.print(
                    fullPacket
                );

                rxIdx = 0;
            }
        }
    }
}

// ============================================================
// CORE 1 TASK:
// LOCAL LCD DISPLAY
// ============================================================

void Task_LCD_Node4(void *pvParameters) {

    Wire.begin(
        PIN_N4_LCD_SDA,
        PIN_N4_LCD_SCL
    );

    lcdN4.begin(16, 4);

    lcdN4.backlight();

    TickType_t xLastWakeTime =
        xTaskGetTickCount();

    const TickType_t xFrequency =
        pdMS_TO_TICKS(500);

    for (;;) {

        vTaskDelayUntil(
            &xLastWakeTime,
            xFrequency
        );

        LocalNode4Data n4Snap = {0};

        if (
            xSemaphoreTake(
                xLocalN4Mutex,
                pdMS_TO_TICKS(20)
            ) == pdTRUE
        ) {

            n4Snap = g_localN4;

            xSemaphoreGive(
                xLocalN4Mutex
            );
        }

        lcdSetRow(0);

        lcdN4.printf(
            "EV:%dL pH:%.1f   ",
            n4Snap.tcVolume,
            n4Snap.ph
        );

        lcdSetRow(1);

        lcdN4.printf(
            "TU:%d EC:%duS     ",
            (int)n4Snap.ntu,
            (int)n4Snap.ec
        );

        lcdSetRow(2);

        lcdN4.printf(
            "ST:%.1fC AT:%.1fC  ",
            n4Snap.subTemp,
            n4Snap.ambTemp
        );

        lcdSetRow(3);

        lcdN4.printf(
            "AH:%.0f%%         ",
            n4Snap.ambHum
        );
    }
}

// ============================================================
// SETUP
// ============================================================

void setup() {

    Serial.begin(115200);

    delay(500);

    loadCalibration();

    xLocalN4Mutex =
        xSemaphoreCreateMutex();

    if (!xLocalN4Mutex) {

        systemFatalTrap(
            "Mutex Allocation"
        );
    }

    BaseType_t r1 =
        xTaskCreatePinnedToCore(
            Task_Sensors_Node4,
            "N4_Sensors",
            4096,
            NULL,
            2,
            NULL,
            1
        );

    BaseType_t r2 =
        xTaskCreatePinnedToCore(
            Task_UART_Node4,
            "N4_UART",
            4096,
            NULL,
            3,
            NULL,
            0
        );

    BaseType_t r3 =
        xTaskCreatePinnedToCore(
            Task_LCD_Node4,
            "N4_LCD",
            3072,
            NULL,
            1,
            NULL,
            1
        );

    if (r1 != pdPASS) {

        systemFatalTrap(
            "N4_Sensors Task"
        );
    }

    if (r2 != pdPASS) {

        systemFatalTrap(
            "N4_UART Task"
        );
    }

    if (r3 != pdPASS) {

        systemFatalTrap(
            "N4_LCD Task"
        );
    }
}

// ============================================================
// LOOP
// ============================================================

void loop() {

    vTaskDelete(NULL);
}

