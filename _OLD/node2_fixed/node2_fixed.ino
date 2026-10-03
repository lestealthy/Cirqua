```cpp
#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

// ============================================================
// PIN DEFINITIONS
// ============================================================

#define TEMP_PIN 4
#define DO_PIN   34

#define TRIG_PIN 16
#define ECHO_PIN 17

// DHT11
// CHANGE THIS PIN IF YOUR DHT11 IS CONNECTED ELSEWHERE
#define DHT_PIN 27
#define DHT_TYPE DHT11


// ============================================================
// UART CONFIGURATION
// ============================================================

// ============================================================
// NODE 1 <=> NODE 2
// ============================================================
//
// Node 1 TX = GPIO 26
// Node 2 RX = GPIO 25
//
// Node 2 TX = GPIO 26
// Node 1 RX = GPIO 25
//
// Same UART is now BIDIRECTIONAL.
// ============================================================

HardwareSerial InSerial(2);


// ============================================================
// NODE 2 -> NODE 3
// ============================================================
//
// Node 2 TX = GPIO 33
// Node 3 RX = GPIO 32
//
// ============================================================

HardwareSerial OutSerial(1);


// ============================================================
// NODE CONFIGURATION
// ============================================================

const String NODE_NAME = "node2";


// ============================================================
// TANK DIMENSIONS
// ============================================================

const float TANK_HEIGHT = 178.0;
const float TANK_RADIUS = 59.5;


// ============================================================
// DO SENSOR CONFIGURATION
// ============================================================

const float VREF = 5000.0;
const int ADC_RESOLUTION = 4095;

const float TWO_POINT_VOLTAGE = 1000.0;


// ============================================================
// TEMPERATURE SENSOR
// ============================================================

OneWire oneWire(TEMP_PIN);
DallasTemperature sensors(&oneWire);


// ============================================================
// DHT11
// ============================================================

DHT dht(DHT_PIN, DHT_TYPE);


// ============================================================
// NODE 1 BUFFER
// ============================================================

// Raw characters currently being received
String node1RxBuffer = "";


// LAST COMPLETE PACKET RECEIVED FROM NODE 1
//
// Example:
//
// |DO:8|Temp:25|TBV:123|node1:1
//
// This value stays unchanged until another complete
// Node 1 packet arrives.

String lastNode1Data = "";


// ============================================================
// SENSOR VALUES
// ============================================================

int currentTemp = -127;
int currentDO = 0;
int currentVolume = -1;

// DHT11 values
int currentAmbientTemp = -127;
int currentHumidity = -1;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    // --------------------------------------------------------
    // Node 1 <=> Node 2
    // --------------------------------------------------------

    InSerial.begin(
        9600,
        SERIAL_8N1,
        25,     // Node 2 RX
        26      // Node 2 TX
    );


    // --------------------------------------------------------
    // Node 2 -> Node 3
    // --------------------------------------------------------

    OutSerial.begin(
        9600,
        SERIAL_8N1,
        32,     // RX
        33      // TX
    );


    // --------------------------------------------------------
    // DS18B20
    // --------------------------------------------------------

    sensors.begin();

    sensors.setWaitForConversion(true);


    // --------------------------------------------------------
    // DHT11
    // --------------------------------------------------------

    dht.begin();


    // --------------------------------------------------------
    // Ultrasonic
    // --------------------------------------------------------

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);

    digitalWrite(TRIG_PIN, LOW);


    // --------------------------------------------------------
    // ADC
    // --------------------------------------------------------

    analogReadResolution(12);


    // --------------------------------------------------------
    // Startup message
    // --------------------------------------------------------

    Serial.println();
    Serial.println("================================");
    Serial.println("NODE 2 INITIALIZED");
    Serial.println("================================");

    Serial.println("NODE 1 <=> NODE 2");
    Serial.println("Baud: 9600");
    Serial.println("Node 1 TX -> Node 2 RX: GPIO26 -> GPIO25");
    Serial.println("Node 2 TX -> Node 1 RX: GPIO26 -> GPIO25");

    Serial.println();

    Serial.println("NODE 2 -> NODE 3");
    Serial.println("Baud: 9600");
    Serial.println("Node 2 TX: GPIO33");
    Serial.println("Node 3 RX: GPIO32");

    Serial.println();

    Serial.print("DHT11 GPIO: ");
    Serial.println(DHT_PIN);

    Serial.println();

    Serial.println("Last-known Node 1 buffering: ENABLED");
    Serial.println("Reverse Node 2 -> Node 1 data: ENABLED");
    Serial.println("================================");
}


// ============================================================
// READ NODE 1 UART
// ============================================================
//
// Node 1 is expected to send:
//
// |DO:8|Temp:25|TBV:123|node1:1;
//
// followed by '\n' if using Serial.println().
//
// IMPORTANT:
//
// We do NOT validate the contents.
//
// Whatever complete packet Node 1 sends is accepted.
//
// This function NEVER waits for Node 1.
// It only reads whatever bytes are already available.
//
// ============================================================

void readNode1UART()
{
    while (InSerial.available())
    {
        char c = InSerial.read();


        // ----------------------------------------------------
        // END OF PACKET
        // ----------------------------------------------------

        if (c == '\n')
        {
            node1RxBuffer.trim();


            if (node1RxBuffer.length() > 0)
            {
                // ------------------------------------------------
                // NEW VALID COMPLETE NODE 1 PACKET
                // ------------------------------------------------

                lastNode1Data = node1RxBuffer;


                Serial.println();
                Serial.println(">>> NEW NODE 1 DATA RECEIVED <<<");
                Serial.print("Node 1: ");
                Serial.println(lastNode1Data);
                Serial.println("================================");
            }


            // Start collecting the next packet
            node1RxBuffer = "";
        }


        // ----------------------------------------------------
        // IGNORE CR
        // ----------------------------------------------------

        else if (c == '\r')
        {
            continue;
        }


        // ----------------------------------------------------
        // NORMAL CHARACTER
        // ----------------------------------------------------

        else
        {
            node1RxBuffer += c;


            // Prevent runaway buffer
            if (node1RxBuffer.length() > 250)
            {
                node1RxBuffer = "";

                Serial.println(
                    "WARNING: Node 1 RX buffer overflow"
                );
            }
        }
    }
}


// ============================================================
// READ DS18B20 TEMPERATURE
// ============================================================

int readTemperature()
{
    sensors.requestTemperatures();

    float tempC = sensors.getTempCByIndex(0);


    if (tempC == DEVICE_DISCONNECTED_C)
    {
        Serial.println(
            "WARNING: DS18B20 disconnected!"
        );

        return -127;
    }


    return (int)round(tempC);
}


// ============================================================
// READ DHT11
// ============================================================
//
// DHT11 provides:
//
// - Ambient temperature
// - Ambient humidity
//
// Both are integer values because DHT11 itself has
// limited resolution.
//
// ============================================================

void readDHT11()
{
    float humidity = dht.readHumidity();
    float ambientTemp = dht.readTemperature();


    if (isnan(humidity) || isnan(ambientTemp))
    {
        Serial.println(
            "WARNING: DHT11 read failed!"
        );

        currentAmbientTemp = -127;
        currentHumidity = -1;

        return;
    }


    currentAmbientTemp = (int)round(ambientTemp);
    currentHumidity = (int)round(humidity);
}


// ============================================================
// READ DISSOLVED OXYGEN
// ============================================================

int readDissolvedOxygen(int temp)
{
    int rawAdc = analogRead(DO_PIN);


    float doVoltage =
        rawAdc *
        (VREF / (float)ADC_RESOLUTION);


    const float saturationDoAt25C = 8.26;


    float compFactor =
        1.0 +
        (temp - 25.0) *
        (-0.02);


    float dissolvedOxygen =
        (doVoltage / TWO_POINT_VOLTAGE) *
        saturationDoAt25C *
        compFactor;


    if (dissolvedOxygen < 0)
        dissolvedOxygen = 0;


    return (int)dissolvedOxygen;
}


// ============================================================
// READ TANK VOLUME
// ============================================================

int readTankVolume()
{
    // --------------------------------------------------------
    // Trigger ultrasonic pulse
    // --------------------------------------------------------

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(5);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(20);

    digitalWrite(TRIG_PIN, LOW);


    // --------------------------------------------------------
    // Read echo
    // --------------------------------------------------------

    unsigned long duration =
        pulseIn(
            ECHO_PIN,
            HIGH,
            20000
        );


    // --------------------------------------------------------
    // No echo
    // --------------------------------------------------------

    if (duration == 0)
    {
        Serial.println(
            "WARNING: Ultrasonic sensor timeout"
        );

        return -1;
    }


    // --------------------------------------------------------
    // Distance
    // --------------------------------------------------------

    float distance =
        duration *
        0.0343 /
        2.0;


    // --------------------------------------------------------
    // Water height
    // --------------------------------------------------------

    float waterHeight =
        TANK_HEIGHT -
        distance;


    if (waterHeight < 0)
        waterHeight = 0;


    if (waterHeight > TANK_HEIGHT)
        waterHeight = TANK_HEIGHT;


    // --------------------------------------------------------
    // Cylinder volume
    // --------------------------------------------------------

    float volume =
        PI *
        TANK_RADIUS *
        TANK_RADIUS *
        waterHeight /
        1000.0;


    return (int)round(volume);
}


// ============================================================
// BUILD NODE 2 DATA
// ============================================================
//
// Existing fields are preserved:
//
// |DO:...|Temp:...|TBV:...|node2:1
//
// New DHT11 fields:
//
// |AT:...|AH:...
//
// AT = Ambient Temperature
// AH = Ambient Humidity
//
// ============================================================

String buildLocalSensorPacket()
{
    String packet = "";

    // Existing DO
    packet += "|DO:";
    packet += String(currentDO);

    // Existing DS18B20 temperature
    packet += "|Temp:";
    packet += String(currentTemp);

    // Existing Tank B volume
    packet += "|TBV:";
    packet += String(currentVolume);

    // DHT11 ambient temperature
    packet += "|AT:";
    packet += String(currentAmbientTemp);

    // DHT11 ambient humidity
    packet += "|AH:";
    packet += String(currentHumidity);

    // Node identifier
    packet += "|";
    packet += NODE_NAME;
    packet += ":1";

    return packet;
}


// ============================================================
// BUILD FINAL PACKET
// ============================================================
//
// If Node 1 has been received:
//
// |NODE1 DATA|DO:...|Temp:...|TBV:...|AT:...|AH:...|node2:1;
//
// If Node 1 has NEVER been received:
//
// |DO:...|Temp:...|TBV:...|AT:...|AH:...|node2:1;
//
// ============================================================

String buildFinalPacket()
{
    String localSensors =
        buildLocalSensorPacket();


    String finalPacket = "";


    // --------------------------------------------------------
    // Add LAST KNOWN Node 1 data
    // --------------------------------------------------------

    if (lastNode1Data.length() > 0)
    {
        finalPacket += lastNode1Data;
    }


    // --------------------------------------------------------
    // Add Node 2 data
    // --------------------------------------------------------

    finalPacket += localSensors;


    // --------------------------------------------------------
    // Final terminator
    // --------------------------------------------------------

    finalPacket += ";";


    return finalPacket;
}


// ============================================================
// SEND NODE 2 DATA BACK TO NODE 1
// ============================================================
//
// Node 2 uses the same UART used to receive Node 1.
//
// GPIO26 = TX from Node 2
// GPIO25 = RX to Node 2
//
// This sends the Node 2 local sensor packet only.
//
// Example:
//
// |DO:8|Temp:25|TBV:123|AT:27|AH:61|node2:1;
//
// ============================================================

void sendDataToNode1()
{
    String reversePacket =
        buildLocalSensorPacket();

    InSerial.println(reversePacket);


    Serial.println();
    Serial.println("<<< NODE 2 DATA SENT BACK TO NODE 1 >>>");
    Serial.print("Node 2 -> Node 1: ");
    Serial.println(reversePacket);
    Serial.println("================================");
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 1. CHECK NODE 1 UART
    // ========================================================
    //
    // This happens FIRST.
    //
    // If Node 1 sent something since the previous loop,
    // it gets stored immediately.
    //
    // If Node 1 sent nothing:
    //
    // lastNode1Data remains unchanged.
    //
    // ========================================================

    readNode1UART();


    // ========================================================
    // 2. READ NODE 2 SENSORS
    // ========================================================

    currentTemp =
        readTemperature();


    currentDO =
        readDissolvedOxygen(
            currentTemp
        );


    currentVolume =
        readTankVolume();


    // ========================================================
    // 3. READ DHT11
    // ========================================================

    readDHT11();


    // ========================================================
    // 4. BUILD FINAL PACKET
    // ========================================================

    String outboundData =
        buildFinalPacket();


    // ========================================================
    // 5. DEBUG
    // ========================================================

    Serial.println();
    Serial.println("------------- NODE 2 DEBUG -------------");


    Serial.print("Buffered Node 1 Data: ");

    if (lastNode1Data.length() > 0)
    {
        Serial.println(lastNode1Data);
    }
    else
    {
        Serial.println("NONE YET");
    }


    Serial.print("Node 2 Data: ");
    Serial.println(
        buildLocalSensorPacket()
    );


    Serial.print("Final packet: ");
    Serial.println(outboundData);


    Serial.println(
        "----------------------------------------"
    );


    // ========================================================
    // 6. SEND FINAL DATA TO NODE 3
    // ========================================================

    OutSerial.println(outboundData);


    // ========================================================
    // 7. SEND NODE 2 DATA BACK TO NODE 1
    // ========================================================

    sendDataToNode1();


    // ========================================================
    // 8. REQUIRED 20 ms DELAY
    // ========================================================

    delay(20);
}
```
