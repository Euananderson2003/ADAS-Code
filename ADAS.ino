#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// === Debug Pin Setup ===
const int debugPin = 31;
bool debugEnabled = false;
#define debugPrint(x)   if (debugEnabled) Serial.print(x)
#define debugPrintln(x) if (debugEnabled) Serial.println(x)

// === Bluetooth ===
SoftwareSerial bluetooth(10, 11);   // RX, TX

// === Mode Jumper ===
const int modePin = 30;             // LOW = 2 sensors, HIGH = 3 sensors

// === Ultrasonic Pins ===
const int trigPinsRear[3]  = {12, 8, 6};
const int echoPinsRear[3]  = {13, 9, 7};
const int trigPinsFront[3] = {18, 22, 24};
const int echoPinsFront[3] = {19, 23, 25};

// === Blind‑spot Pins ===
const int blindTrigL = 26;
const int blindEchoL = 27;
const int blindTrigR = 28;
const int blindEchoR = 29;

// === IR Line‑sensor Pins (interrupt) ===
const int irSensorPinL = 2;
const int irSensorPinR = 3;

// === Temperature Sensor ===
const int tempPin = 5;
OneWire oneWire(tempPin);
DallasTemperature sensors(&oneWire);

// === Runtime Variables ===
const int MAX_SENSORS = 6;
int  distances[MAX_SENSORS] = {0};
int  blindLeft  = 999;
int  blindRight = 999;
int  tempC      = 0;
int  sensorCount = 2;               // updated by jumper

volatile int irLeft  = 0;
volatile int irRight = 0;

// Streaming flags – permanently ON ==================================
const bool rearStreaming  = true;
const bool frontStreaming = true;

String btBuffer = "";                // still used for DEBUG commands
unsigned long lastSensorTime   = 0;
const unsigned long sensorInterval = 100;   // ms, 10 Hz stream

// ===================================================================
void setup() {
  Serial.begin(38400);
  bluetooth.begin(38400);

  pinMode(debugPin, INPUT_PULLUP);
  debugEnabled = (digitalRead(debugPin) == LOW);
  if (debugEnabled) Serial.println(F("Debug Mode ENABLED"));

  pinMode(modePin, INPUT_PULLUP);

  // IR sensors ===================================================
  pinMode(irSensorPinL, INPUT_PULLUP);
  pinMode(irSensorPinR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(irSensorPinL), isrIRLeft,  CHANGE);
  attachInterrupt(digitalPinToInterrupt(irSensorPinR), isrIRRight, CHANGE);

  // Ultrasonic TX/RX setup ==================================
  for (int i = 0; i < 3; i++) {
    pinMode(trigPinsRear[i],  OUTPUT);
    pinMode(echoPinsRear[i],  INPUT);
    pinMode(trigPinsFront[i], OUTPUT);
    pinMode(echoPinsFront[i], INPUT);
  }

  // Blind‑spot ultrasonic ===========================================
  pinMode(blindTrigL, OUTPUT);
  pinMode(blindEchoL, INPUT);
  pinMode(blindTrigR, OUTPUT);
  pinMode(blindEchoR, INPUT);

  sensors.begin();

  debugPrintln(F("Ultrasonic sensor unit ready – streaming begins."));
}

// ===================================================================
void loop() {
  handleDebugCommands();      // only DEBUG:1 / DEBUG:0 recognised
  jumperCheck();

  if (millis() - lastSensorTime >= sensorInterval) {
    readUltrasonics();
    readBlindSpots();
    tempSensor();
    dataWrite();
    lastSensorTime = millis();
  }
}

// ===================================================================
// Only DEBUG commands are parsed; REAR/FRONT commands removed.
void handleDebugCommands() {
  while (bluetooth.available()) {
    char c = bluetooth.read();
    if (c == '\n') {
      btBuffer.trim();
      if (btBuffer == "DEBUG:1") {
        debugEnabled = true;  bluetooth.println("DEBUG_ON");
      } else if (btBuffer == "DEBUG:0") {
        debugEnabled = false; bluetooth.println("DEBUG_OFF");
      }
      btBuffer = "";
    } else {
      btBuffer += c;
    }
  }
}

// ===================================================================
void jumperCheck() {
  sensorCount = (digitalRead(modePin) == LOW) ? 2 : 3;
}

// ================= Ultrasonic distances (rear/front) =================
void readUltrasonics() {
  for (int i = 0; i < sensorCount; i++) {
    int rIndex = (sensorCount == 2) ? (i == 0 ? 0 : 2) : i;   // same mapping as original
    distances[i] = getDistance(trigPinsRear[rIndex], echoPinsRear[rIndex]);

    int fIndex = (sensorCount == 2) ? (i == 0 ? 0 : 2) : i;
    distances[i + sensorCount] = getDistance(trigPinsFront[fIndex], echoPinsFront[fIndex]);
  }
}

// ================= Blind‑spot ultrasonics =================
void readBlindSpots() {
  blindLeft  = getDistance(blindTrigL, blindEchoL);
  blindRight = getDistance(blindTrigR, blindEchoR);
}

// ================= Get single ultrasonic distance =================
int getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 15000);         // time‑out 15 ms
  int distance = (duration == 0) ? 999 : duration * 0.034 / 2;
  return (distance > 600) ? 666 : distance;              // 666 = "clear" flag
}

// ================= Temperature sensor =================
void tempSensor() {
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);
}

// ================= IR line‑sensor ISRs =================
void isrIRLeft()  { irLeft  = (digitalRead(irSensorPinL) == LOW) ? 1 : 0; }
void isrIRRight() { irRight = (digitalRead(irSensorPinR) == LOW) ? 1 : 0; }

// ===================================================================
void dataWrite() {
  bool tempError = (tempC == DEVICE_DISCONNECTED_C);

  uint8_t buf[32];
  uint8_t idx = 0;

  buf[idx++] = 0xAA;              // HEADER
  buf[idx++] = 0x02;              // TYPE (sensor data)
  buf[idx++] = 0x03;              // FLAGS – rear + front streaming ON

  // Rear distances ===================================================
  for (int i = 0; i < 3; i++) {
    uint16_t val = (i < sensorCount) ? distances[i] : 0xFFFF;
    buf[idx++] = val & 0xFF;  buf[idx++] = val >> 8;
  }
  // Front distances ===================================================
  for (int i = 0; i < 3; i++) {
    uint16_t val = (i < sensorCount) ? distances[i + sensorCount] : 0xFFFF;
    buf[idx++] = val & 0xFF;  buf[idx++] = val >> 8;
  }

  // Blind‑spot ===================================================
  buf[idx++] = blindLeft  & 0xFF; buf[idx++] = blindLeft  >> 8;
  buf[idx++] = blindRight & 0xFF; buf[idx++] = blindRight >> 8;

  // Temperature ===================================================
  uint16_t t = tempError ? 0xFFFF : (uint16_t)tempC;
  buf[idx++] = t & 0xFF;  buf[idx++] = t >> 8;

  // IR flags ===================================================
  buf[idx++] = (irLeft ? 1 : 0) | (irRight ? 2 : 0);

  buf[idx++] = 0x55;              // TAIL

  bluetooth.write(buf, idx);

  if (debugEnabled) {
    Serial.print(F("Sent: "));
    for (uint8_t i = 0; i < idx; i++) { Serial.print(buf[i], HEX); Serial.print(' '); }
    Serial.println();
  }
}
