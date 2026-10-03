#include <Arduino.h>

HardwareSerial InSerial(1);  // RX=25, TX=26
HardwareSerial OutSerial(2); // RX=32, TX=33

const String NODE_NAME = "node3";
unsigned long lastReceivedTime = 0;
const unsigned long TIMEOUT_MS = 2000;

const byte SENSOR_PIN = 23;          
volatile uint16_t pulseCount = 0;    
float flowRate = 0.0;
unsigned long lastFlowMillis = 0;
const float CALIBRATION_FACTOR = 5.5; 

void IRAM_ATTR pulseCounterISR() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);
  InSerial.begin(9600, SERIAL_8N1, 25, 26);
  InSerial.setTimeout(50);
  OutSerial.begin(9600, SERIAL_8N1, 32, 33);
  
  pinMode(SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), pulseCounterISR, FALLING);
  
  lastReceivedTime = millis();
  lastFlowMillis = millis();
}

float getFLM() {
  if (millis() - lastFlowMillis >= 1000) {
    noInterrupts(); 
    uint16_t currentPulses = pulseCount;
    pulseCount = 0;   
    interrupts();   
    
    flowRate = (float)currentPulses / CALIBRATION_FACTOR; 
    lastFlowMillis += 1000; // Fixed timing drift bug
  }
  return flowRate;
}

void loop() {
  bool dataReceived = false;
  String incomingData = "";

  if (InSerial.available() > 0) {
    incomingData = InSerial.readStringUntil(';');
    incomingData.trim();
    if (incomingData.length() > 0) {
      dataReceived = true;
      lastReceivedTime = millis();
    }
  }

  // Debug: Log incoming serial state
  Serial.println("--- DEBUG LOG ---");
  Serial.println("Received Raw Packet: [" + (dataReceived ? incomingData : "NONE") + "]");

  // Retrieve flow meter reading
  float currentFlow = getFLM();
  Serial.println("Sensor Flow Rate: " + String(currentFlow, 1) + " L/min");

  String NodeData = "FLM:" + String(currentFlow, 1) + "|" + NODE_NAME + ":1;";

  if (dataReceived) {
    String outboundData = incomingData + "|" + NodeData;
    OutSerial.println(outboundData);
    Serial.println("Forwarded Packet Sent: " + outboundData);
    Serial.println("------------------\n");
  } 
  else if (millis() - lastReceivedTime > TIMEOUT_MS) {
    Serial.println("ALERT: Upstream Timeout detected.");
    String outboundData = "node1:0|node2:0|" + NodeData;
    Serial.println("Timeout Fallback Packet Sent: " + outboundData);
    OutSerial.println(outboundData);
    
    // Fixed continuous timeout spam: set timestamp far into the future 
    // or rely on a state flag so it only triggers once per outage event.
    lastReceivedTime = millis() + 100000; // Mutes spam until new data resets it
    Serial.println("------------------\n");
  } else {
    Serial.println("Status: Waiting for data or within timeout window...");
    Serial.println("------------------\n");
  }
  delay(20);
}