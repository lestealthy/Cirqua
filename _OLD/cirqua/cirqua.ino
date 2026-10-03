#include <Arduino.h>
#include <SoftwareSerial.h>

/*
  Cirqua_Node5_Master.ino
  USB Serial protocol:
    - SET_JP<num>:1|0  -> control JP1..JP20 (LOW=ON, HIGH=OFF). Reply: ACK
    - READ_JP<num>     -> reply: JP<num>:1 (ON/LOW) or JP<num>:0 (OFF/HIGH)
    - READ_SENSORS     -> reply: one JSON line parsed from Node 4 telemetry stream
    - KILL             -> all JPs OFF (HIGH), sensors report zeros; reply: ACK
    - RESET            -> clear KILL, all JPs OFF (HIGH); reply: ACK
*/

// Software Serial on Mega: RX=Pin 2, TX=Pin 3 (Connect Node 4 TX to Mega Pin 2)
SoftwareSerial Node4Serial(10, 11); 

// ---- JP pin mapping (Arduino Mega) ----
static const uint8_t JP_PINS[20] = {
  27, 26, 28, 29, 30,
  31, 37, 36, 35, 34,
  32, 33, 38, 39, 40,
  41, 42, 43, 44, 45
};

bool killed = false;
String lastChainData = ""; // Cache for raw incoming stream string from Node 4

// --- Global Sensor State Captured from Node Chain ---
String val_TAV  = "0";
String val_DO   = "0";
String val_Temp = "0.0";
String val_TBV  = "0";
String val_FLM  = "0.0";
String val_PH   = "0.0";
String val_Turb = "0";
String val_EC   = "0.0";
String val_TCV  = "0";

// ----------- Utilities -----------
static inline void writeAck() {
  Serial.println(F("ACK"));
}

static inline void writeErr(const __FlashStringHelper *msg) {
  Serial.print(F("ERR:"));
  Serial.println(msg);
}

static inline bool jpIndexFromNumber(int jpNumber, int &idxOut) {
  if (jpNumber < 1 || jpNumber > 20) return false;
  idxOut = jpNumber - 1;
  return true;
}

static inline uint8_t readRelayLogicalState(int idx) {
  return (digitalRead(JP_PINS[idx]) == LOW) ? 1 : 0;
}

static inline void setRelayByLogicalState(int idx, uint8_t on) {
  digitalWrite(JP_PINS[idx], on ? LOW : HIGH);
}

static void allRelaysOff() {
  for (int i = 0; i < 20; ++i) digitalWrite(JP_PINS[i], HIGH);
}

// Helper to isolate specific field values from the tokenized string payload
String extractValue(const String &stream, const String &key) {
  int keyIndex = stream.indexOf(key + ":");
  if (keyIndex == -1) return "";
  
  int startValue = keyIndex + key.length() + 1;
  int endValue = stream.indexOf('|', startValue);
  if (endValue == -1) {
    endValue = stream.indexOf(';', startValue);
  }
  
  if (endValue == -1) return stream.substring(startValue);
  return stream.substring(startValue, endValue);
}

// Parses the token array coming out of Node 4
void parseChainTelemetry(String data) {
  data.trim();
  if (data.length() == 0) return;

  String tmp;
  
  if ((tmp = extractValue(data, "TAV"))  != "") val_TAV  = tmp;
  if ((tmp = extractValue(data, "DO"))   != "") val_DO   = tmp;
  if ((tmp = extractValue(data, "Temp")) != "") val_Temp = tmp;
  if ((tmp = extractValue(data, "TBV"))  != "") val_TBV  = tmp;
  if ((tmp = extractValue(data, "FLM"))  != "") val_FLM  = tmp;
  if ((tmp = extractValue(data, "pH"))   != "") val_PH   = tmp;
  if ((tmp = extractValue(data, "Turb")) != "") val_Turb = tmp;
  if ((tmp = extractValue(data, "EC"))   != "") val_EC   = tmp;
  if ((tmp = extractValue(data, "TCV"))  != "") val_TCV  = tmp;
}

// ==================================================
// =============== SENSOR OUTPUT ====================
// ==================================================

static void writeSensorsJson() {
  if (killed) {
    //Serial.println(F("{\"CO\":0.00,\"DO\":0.00,\"PH\":0.00,\"TAV\":0,\"TBV\":0,\"TCV\":0,\"Temp\":0.00,\"Turp\":0.00}"));
    Serial.println(F("{\"CO\":0.00,\"DO\":0.00,\"PH\":0.00,\"TAV\":0,\"TBV\":0,\"TCV\":0,\"Temp\":0.00,\"Turp\":0.00,\"FLM\":0.00}"));
    return;
  }

  // Formatting directly as requested to match the upstream dynamic string
  Serial.print(F("{\"CO\":"));    Serial.print(val_EC);
  Serial.print(F(",\"DO\":"));   Serial.print(val_DO);
  Serial.print(F(",\"PH\":"));   Serial.print(val_PH);
  Serial.print(F(",\"TAV\":"));  Serial.print(val_TAV);
  Serial.print(F(",\"TBV\":"));  Serial.print(val_TBV);
  Serial.print(F(",\"TBV\":"));  Serial.print(val_TBV);
  Serial.print(F(",\"TCV\":"));  Serial.print(val_TCV);
  Serial.print(F(",\"Temp\":")); Serial.print(val_Temp);
  Serial.print(F(",\"Turp\":")); Serial.print(val_Turb);
  Serial.print(F(",\"FLM\":"));  Serial.print(val_FLM);
  Serial.println(F("}"));
}

// ==================================================
// =============== COMMAND HANDLERS =================
// ==================================================

static void handleSetJp(const String &cmd) {
  int p = cmd.indexOf('P');
  int c = cmd.indexOf(':');
  if (p < 0 || c < 0 || c <= p + 1) {
    writeErr(F("BAD_FMT"));
    return;
  }

  int num = cmd.substring(p + 1, c).toInt();
  int idx;
  if (!jpIndexFromNumber(num, idx)) {
    writeErr(F("BAD_JP"));
    return;
  }

  int val = cmd.substring(c + 1).toInt();
  if (val != 0 && val != 1) {
    writeErr(F("BAD_VAL"));
    return;
  }

  setRelayByLogicalState(idx, (uint8_t)val);
  writeAck();
}

static void handleReadJp(const String &cmd) {
  int p = cmd.indexOf('P');
  if (p < 0) {
    writeErr(F("BAD_FMT"));
    return;
  }

  int num = cmd.substring(p + 1).toInt();
  int idx;
  if (!jpIndexFromNumber(num, idx)) {
    writeErr(F("BAD_JP"));
    return;
  }

  Serial.print(F("JP"));
  Serial.print(num);
  Serial.print(F(":"));
  Serial.println(readRelayLogicalState(idx));
}

static void handleKill() {
  killed = true;
  allRelaysOff();
  writeAck();
}

static void handleReset() {
  writeAck();
  killed = false;
  allRelaysOff();
}

static void handleReadSensors() {
  writeSensorsJson();
}

static void processCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  cmd.toUpperCase();

  if (cmd.startsWith(F("SET_JP"))) {
    handleSetJp(cmd);
    return;
  }
  if (cmd.startsWith(F("READ_JP"))) {
    handleReadJp(cmd);
    return;
  }
  if (cmd == F("READ_SENSORS")) {
    handleReadSensors();
    return;
  }
  if (cmd == F("KILL")) {
    handleKill();
    return;
  }
  if (cmd == F("RESET")) {
    handleReset();
    return;
  }

  writeErr(F("UNKNOWN"));
}

// ==================================================
// ================= SETUP / LOOP ===================
// ==================================================

void setup() {
  Serial.begin(115200);
  
  // Software Serial setup for interacting with Node 4 
  Node4Serial.begin(9600);
  Node4Serial.setTimeout(30);

  while (!Serial) { ; }

  for (int i = 0; i < 20; ++i) {
    pinMode(JP_PINS[i], OUTPUT);
    digitalWrite(JP_PINS[i], HIGH);  // OFF (active-low setup)
  }
}

void loop() {
  static String usbBuffer;

  // 1. Process Background Data stream arriving from Node 4
  if (Node4Serial.available() > 0) {
    String tempStream = Node4Serial.readStringUntil('\n');
    tempStream.trim();
    
    // Discard any garbage line headers if present
    while (tempStream.length() > 0 && (tempStream[0] < 32 || tempStream[0] > 126)) {
      tempStream.remove(0, 1);
    }
    
    if (tempStream.length() > 0) {
      lastChainData = tempStream;
      parseChainTelemetry(lastChainData); // Unpack the tokens into individual metrics
    }
  }

  // 2. Process Controller Commands coming via Primary USB Port
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (usbBuffer.length()) {
        processCommand(usbBuffer);
        usbBuffer = "";
      }
    } else if (usbBuffer.length() < 128) {
      usbBuffer += c;
    }
  }
}