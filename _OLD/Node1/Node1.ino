```cpp
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ============================================================
// NODE 1 TANK SENSOR
// ============================================================

#define TRIG_PIN 2
#define ECHO_PIN 17


// ============================================================
// UART CONFIGURATION
// ============================================================

// ------------------------------------------------------------
// NODE 1 -> NODE 2
// ------------------------------------------------------------
//
// Existing connection:
//
// Node 1 TX = GPIO 33
// Node 2 RX = GPIO 25
//
// ------------------------------------------------------------

HardwareSerial OutSerial(2);


// ------------------------------------------------------------
// NODE 2 -> NODE 1
// ------------------------------------------------------------
//
// Reverse communication:
//
// Node 2 TX = GPIO 26
// Node 1 RX = GPIO 25
//
// Node 1 TX = GPIO 26
// Node 2 RX = GPIO 25
//
// UART1 is used here.
// ------------------------------------------------------------

HardwareSerial InSerial(1);


// ============================================================
// LCD
// ============================================================

LiquidCrystal_I2C lcd(0x27, 16, 4);


// ============================================================
// NODE CONFIGURATION
// ============================================================

const String NODE_NAME = "node1";


// ============================================================
// TANK DIMENSIONS
// ============================================================

const float TANK_HEIGHT = 260.0;
const float TANK_RADIUS = 110.0;


// ============================================================
// NODE 2 RECEIVE BUFFER
// ============================================================

String node2RxBuffer = "";


// ============================================================
// LAST RECEIVED NODE 2 VALUES
// ============================================================

// Dissolved oxygen
float node2DO = -1.0;

// DS18B20 submerged temperature
float node2WaterTemp = -127.0;

// Node 2 / Feeding Tank volume
int node2Volume = -1;

// DHT11 ambient temperature
float node2AmbientTemp = -127.0;

// DHT11 humidity
float node2Humidity = -1.0;


// ============================================================
// NODE 2 COMMUNICATION STATUS
// ============================================================

// millis() timestamp of the last complete Node 2 packet
unsigned long lastNode2Packet = 0;

// Consider Node 2 communication lost after this time
const unsigned long NODE2_TIMEOUT = 3000;


// ============================================================
// NODE 2 PACKET VALIDITY
// ============================================================

bool node2PacketValid = false;


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);


    // ========================================================
    // NODE 1 -> NODE 2
    // ========================================================

    OutSerial.begin(
        9600,
        SERIAL_8N1,
        32,     // RX
        33      // TX
    );


    // ========================================================
    // NODE 2 <=> NODE 1
    // ========================================================
    //
    // GPIO25 = RX
    // GPIO26 = TX
    //
    // This UART is used for receiving Node 2 data.
    //
    // ========================================================

    InSerial.begin(
        9600,
        SERIAL_8N1,
        25,     // RX
        26      // TX
    );


    // ========================================================
    // LCD
    // ========================================================

    lcd.begin(16, 4);
    lcd.backlight();


    // ========================================================
    // ULTRASONIC
    // ========================================================

    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);


    // ========================================================
    // STARTUP
    // ========================================================

    Serial.println();
    Serial.println("================================");
    Serial.println("NODE 1 STARTED");
    Serial.println("================================");

    Serial.println("NODE 1 -> NODE 2");
    Serial.println("Baud: 9600");
    Serial.println("TX: GPIO33");

    Serial.println();

    Serial.println("NODE 2 -> NODE 1");
    Serial.println("Baud: 9600");
    Serial.println("RX: GPIO25");
    Serial.println("TX: GPIO26");

    Serial.println("================================");


    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("Water Monitor");

    lcd.setCursor(0, 1);
    lcd.print("Booting...");

    delay(500);

    lcd.clear();
}


// ============================================================
// READ NODE 1 TANK VOLUME
// ============================================================

int getSensorValue()
{
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);

    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIG_PIN, LOW);


    long duration =
        pulseIn(
            ECHO_PIN,
            HIGH,
            30000
        );


    float distance =
        (duration * 0.0343 / 2.0) - 12.5;


    float waterHeight =
        TANK_HEIGHT - distance;


    if (waterHeight < 0)
        waterHeight = 0;


    float volumeLiters =
        (
            3.14159 *
            TANK_RADIUS *
            TANK_RADIUS *
            waterHeight
        ) / 1000.0;


    return volumeLiters * 2;
}


// ============================================================
// PARSE A NODE 2 FIELD
// ============================================================
//
// Extracts:
//
// DO
// Temp
// TBV
// AT
// AH
//
// Example:
//
// |DO:8|Temp:25|TBV:3500|AT:27|AH:61|node2:1
//
// ============================================================

String getField(
    String packet,
    String fieldName
)
{
    String search =
        "|" + fieldName + ":";


    int start =
        packet.indexOf(search);


    if (start < 0)
        return "";


    start += search.length();


    int end =
        packet.indexOf(
            "|",
            start
        );


    if (end < 0)
    {
        end =
            packet.indexOf(
                ";",
                start
            );
    }


    if (end < 0)
        end = packet.length();


    return packet.substring(
        start,
        end
    );
}


// ============================================================
// PROCESS COMPLETE NODE 2 PACKET
// ============================================================

void processNode2Packet(
    String packet
)
{
    packet.trim();


    Serial.println();
    Serial.println(">>> NODE 2 DATA RECEIVED <<<");
    Serial.println(packet);


    // --------------------------------------------------------
    // DO
    // --------------------------------------------------------

    String doValue =
        getField(
            packet,
            "DO"
        );


    if (doValue.length() > 0)
    {
        node2DO =
            doValue.toFloat();
    }


    // --------------------------------------------------------
    // WATER TEMPERATURE
    // --------------------------------------------------------

    String tempValue =
        getField(
            packet,
            "Temp"
        );


    if (tempValue.length() > 0)
    {
        node2WaterTemp =
            tempValue.toFloat();
    }


    // --------------------------------------------------------
    // FEEDING TANK VOLUME
    // --------------------------------------------------------

    String volumeValue =
        getField(
            packet,
            "TBV"
        );


    if (volumeValue.length() > 0)
    {
        node2Volume =
            volumeValue.toInt();
    }


    // --------------------------------------------------------
    // AMBIENT TEMPERATURE
    // --------------------------------------------------------

    String ambientTempValue =
        getField(
            packet,
            "AT"
        );


    if (ambientTempValue.length() > 0)
    {
        node2AmbientTemp =
            ambientTempValue.toFloat();
    }


    // --------------------------------------------------------
    // AMBIENT HUMIDITY
    // --------------------------------------------------------

    String humidityValue =
        getField(
            packet,
            "AH"
        );


    if (humidityValue.length() > 0)
    {
        node2Humidity =
            humidityValue.toFloat();
    }


    // --------------------------------------------------------
    // CHECK WHETHER ALL REQUIRED DATA EXISTS
    // --------------------------------------------------------

    if (
        doValue.length() > 0 &&
        tempValue.length() > 0 &&
        volumeValue.length() > 0 &&
        ambientTempValue.length() > 0 &&
        humidityValue.length() > 0
    )
    {
        node2PacketValid = true;
        lastNode2Packet = millis();
    }
    else
    {
        node2PacketValid = false;
    }


    // --------------------------------------------------------
    // DEBUG
    // --------------------------------------------------------

    Serial.print("DO: ");
    Serial.println(node2DO);

    Serial.print("Water Temp: ");
    Serial.println(node2WaterTemp);

    Serial.print("Feeding Tank Volume: ");
    Serial.println(node2Volume);

    Serial.print("Ambient Temp: ");
    Serial.println(node2AmbientTemp);

    Serial.print("Humidity: ");
    Serial.println(node2Humidity);

    Serial.println(
        "================================"
    );
}


// ============================================================
// READ NODE 2 UART
// ============================================================
//
// Non-blocking.
//
// Reads whatever is already available.
//
// A complete packet ends with '\n'.
//
// ============================================================

void readNode2UART()
{
    while (InSerial.available())
    {
        char c =
            InSerial.read();


        // ----------------------------------------------------
        // COMPLETE PACKET
        // ----------------------------------------------------

        if (c == '\n')
        {
            node2RxBuffer.trim();


            if (node2RxBuffer.length() > 0)
            {
                processNode2Packet(
                    node2RxBuffer
                );
            }


            node2RxBuffer = "";
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
            node2RxBuffer += c;


            // Prevent runaway buffer
            if (node2RxBuffer.length() > 250)
            {
                node2RxBuffer = "";

                Serial.println(
                    "WARNING: Node 2 RX buffer overflow"
                );
            }
        }
    }
}


// ============================================================
// CHECK NODE 2 STATUS
// ============================================================
//
// IMPORTANT:
//
// This is NOT a real alarm/status flag.
//
// It only determines whether communication/data are valid.
//
// ============================================================

String getStatus()
{
    // No Node 2 packet received yet
    if (!node2PacketValid)
    {
        return "WAITING";
    }


    // Node 2 stopped communicating
    if (
        millis() -
        lastNode2Packet >
        NODE2_TIMEOUT
    )
    {
        return "ERROR";
    }


    // Data exists and is recent
    return "NORMAL";
}


// ============================================================
// PRINT FIXED-WIDTH LCD LINE
// ============================================================
//
// Guarantees that the entire 16-character line is overwritten.
//
// ============================================================

void printLCDLine(
    byte row,
    String text
)
{
    lcd.setCursor(0, row);


    // Trim to 16 characters
    if (text.length() > 16)
    {
        text =
            text.substring(
                0,
                16
            );
    }


    lcd.print(text);


    // Clear remaining characters
    for (
        int i = text.length();
        i < 16;
        i++
    )
    {
        lcd.print(" ");
    }
}


// ============================================================
// UPDATE LCD
// ============================================================

void updateDisplay(
    int collectionVolume
)
{
    // --------------------------------------------------------
    // LINE 1
    // --------------------------------------------------------
    //
    // C12345L F1234L
    //
    // C = Collection Tank
    // F = Feeding Tank
    // --------------------------------------------------------

    String line1 = "";

    line1 += "C";
    line1 += String(collectionVolume);
    line1 += "L";

    line1 += " F";

    if (node2Volume >= 0)
        line1 += String(node2Volume);
    else
        line1 += "----";

    line1 += "L";


    printLCDLine(
        0,
        line1
    );


    // --------------------------------------------------------
    // LINE 2
    // --------------------------------------------------------
    //
    // DO12,3g/L T25,4C
    //
    // 16 characters maximum
    // --------------------------------------------------------

    String line2 = "";

    line2 += "DO";

    if (node2DO >= 0)
    {
        line2 += String(
            node2DO,
            1
        );
    }
    else
    {
        line2 += "--";
    }

    // Decimal separator requested by user
    line2.replace(
        ".",
        ","
    );

    line2 += "g/L";

    line2 += " T";

    if (node2WaterTemp > -100)
    {
        line2 += String(
            node2WaterTemp,
            1
        );
    }
    else
    {
        line2 += "--";
    }

    line2.replace(
        ".",
        ","
    );

    line2 += "C";


    printLCDLine(
        1,
        line2
    );


    // --------------------------------------------------------
    // LINE 3
    // --------------------------------------------------------
    //
    // AT26,1C H58%
    // --------------------------------------------------------

    String line3 = "";

    line3 += "AT";

    if (node2AmbientTemp > -100)
    {
        line3 += String(
            node2AmbientTemp,
            1
        );
    }
    else
    {
        line3 += "--";
    }

    line3.replace(
        ".",
        ","
    );

    line3 += "C H";

    if (node2Humidity >= 0)
    {
        line3 += String(
            node2Humidity,
            0
        );
    }
    else
    {
        line3 += "--";
    }

    line3 += "%";


    printLCDLine(
        2,
        line3
    );


    // --------------------------------------------------------
    // LINE 4
    // --------------------------------------------------------

    String statusLine =
        "STATUS: " +
        getStatus();


    printLCDLine(
        3,
        statusLine
    );
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    // ========================================================
    // 1. READ NODE 2 DATA
    // ========================================================

    readNode2UART();


    // ========================================================
    // 2. READ COLLECTION TANK
    // ========================================================

    int collectionVolume =
        getSensorValue();


    // ========================================================
    // 3. SEND COLLECTION TANK DATA TO NODE 2
    // ========================================================

    String outboundData =
        "TAV:" +
        String(collectionVolume) +
        "|" +
        NODE_NAME +
        ":1;";


    OutSerial.println(
        outboundData
    );


    Serial.println(
        "Sent: " +
        outboundData
    );


    // ========================================================
    // 4. UPDATE LCD
    // ========================================================

    updateDisplay(
        collectionVolume
    );


    // ========================================================
    // 5. LOOP DELAY
    // ========================================================

    delay(500);
}
```
