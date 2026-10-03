#include <Arduino.h>

#define INTERNODE_BAUD 9600

// Hardware Pin Assignments
#define PIN_FLOW_SENSOR   23
#define PIN_UP_UART_RX    25
#define PIN_UP_UART_TX    26
#define PIN_DOWN_UART_RX  32
#define PIN_DOWN_UART_TX  33

#define FLOW_CAL_FACTOR   5.5f
#define TIMEOUT_MS        2000

// Interrupt Pulse Counter
static volatile uint32_t g_pulseCount = 0;
static portMUX_TYPE g_pulseMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR pulseISR() {
    portENTER_CRITICAL_ISR(&g_pulseMux);
    g_pulseCount++;
    portEXIT_CRITICAL_ISR(&g_pulseMux);
}

HardwareSerial UpstreamSerial(1);
HardwareSerial DownstreamSerial(2);

struct FlowData {
    float flowRate;
    bool isValid;
};

static FlowData g_flowData = {0.0f, false};
static SemaphoreHandle_t xFlowMutex = NULL;

void systemFatalTrap(const char* taskName) {
    Serial.printf("[FATAL ERROR] Node 3 Task Failed: %s\n", taskName);
    while (1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

// ============================================================
// TASK: 1-SECOND PERIODIC FLOW METER CALCULATION (CORE 1)
// ============================================================
void Task_Flow_Node3(void *pvParameters) {
    pinMode(PIN_FLOW_SENSOR, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_FLOW_SENSOR), pulseISR, FALLING);

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t pulses = 0;
        portENTER_CRITICAL(&g_pulseMux);
        pulses = g_pulseCount;
        g_pulseCount = 0;
        portEXIT_CRITICAL(&g_pulseMux);

        FlowData currentFlow;
        currentFlow.flowRate = (float)pulses / FLOW_CAL_FACTOR; // L/min
        currentFlow.isValid = true;

        if (xSemaphoreTake(xFlowMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_flowData = currentFlow;
            xSemaphoreGive(xFlowMutex);
        }
    }
}

// Frame Integrity Check
bool validateUpstreamFrame(const char* frame) {
    if (!frame) return false;
    return (strstr(frame, "TAV:") || strstr(frame, "|TAV:")) &&
           strstr(frame, "|node1:") &&
           strstr(frame, "|DO:")   &&
           strstr(frame, "|node2:");
}

// ============================================================
// TASK: DUAL UART TELEMETRY & FORWARDING (CORE 0)
// ============================================================
void Task_UART_Node3(void *pvParameters) {
    UpstreamSerial.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_UP_UART_RX, PIN_UP_UART_TX);
    DownstreamSerial.begin(INTERNODE_BAUD, SERIAL_8N1, PIN_DOWN_UART_RX, PIN_DOWN_UART_TX);

    char rxBuffer[256];
    size_t rxIdx = 0;
    uint32_t lastUpstreamRxTime = millis();
    bool timeoutAlertSent = false;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        FlowData fSnap = {0.0f, false};
        if (xSemaphoreTake(xFlowMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            fSnap = g_flowData;
            xSemaphoreGive(xFlowMutex);
        }

        bool receivedNewFrame = false;

        // Process incoming stream from Node 2
        while (UpstreamSerial.available() > 0) {
            char c = (char)UpstreamSerial.read();
            if (c == '\n' || c == '\r') continue;

            if (rxIdx < sizeof(rxBuffer) - 1) {
                rxBuffer[rxIdx++] = c;
            } else {
                rxIdx = 0; // Prevent buffer overflow
            }

            if (c == ';') {
                rxBuffer[rxIdx] = '\0';

                if (validateUpstreamFrame(rxBuffer)) {
                    // Strip trailing delimiter
                    if (rxIdx > 0 && rxBuffer[rxIdx - 1] == ';') {
                        rxBuffer[rxIdx - 1] = '\0';
                    }

                    char clusterPacket[320];
                    // Standardized key name FLM for downstream controller compatibility
                    snprintf(clusterPacket, sizeof(clusterPacket), "%s|FLM:%.2f|node3:%d;\n",
                             rxBuffer,
                             fSnap.isValid ? fSnap.flowRate : 0.0f,
                             fSnap.isValid ? 1 : 0);

                    DownstreamSerial.print(clusterPacket);
                    lastUpstreamRxTime = millis();
                    timeoutAlertSent = false;
                    receivedNewFrame = true;
                }
                rxIdx = 0;
            }
        }

        // Upstream Timeout Fallback Transmission
        if (!receivedNewFrame && (millis() - lastUpstreamRxTime > TIMEOUT_MS) && !timeoutAlertSent) {
            char fallbackPacket[128];
            snprintf(fallbackPacket, sizeof(fallbackPacket),
                     "|TAV:0.00|node1:0|DO:0.00|node2:0|FLM:%.2f|node3:%d;\n",
                     fSnap.isValid ? fSnap.flowRate : 0.0f,
                     fSnap.isValid ? 1 : 0);

            DownstreamSerial.print(fallbackPacket);
            timeoutAlertSent = true; // Mute spam until link recovers
        }
    }
}

// ============================================================
// SETUP & MAIN LOOP
// ============================================================
void setup() {
    Serial.begin(115200);

    xFlowMutex = xSemaphoreCreateMutex();
    if (!xFlowMutex) systemFatalTrap("Mutex Allocation");

    BaseType_t r1 = xTaskCreatePinnedToCore(Task_Flow_Node3, "N3_Flow", 3072, NULL, 2, NULL, 1);
    BaseType_t r2 = xTaskCreatePinnedToCore(Task_UART_Node3, "N3_UART", 3072, NULL, 3, NULL, 0);

    if (r1 != pdPASS) systemFatalTrap("N3_Flow Task Creation");
    if (r2 != pdPASS) systemFatalTrap("N3_UART Task Creation");
}

void loop() {
    vTaskDelete(NULL);
}