#include <SoftwareSerial.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ========= Debug Setup =========
const int debugPin = 7;
bool debugEnabled  = false;
#define debugPrint(x)   if (debugEnabled) Serial.print(x)
#define debugPrintln(x) if (debugEnabled) Serial.println(x)

// ========= Bluetooth Setup =========
SoftwareSerial bluetooth(10, 11);            // RX, TX

// ========= Pin Setup =========
const int modePin      = 6;  // LOW = 2 sensors, HIGH = 3 sensors
const int tempPin      = 5;  // DS18B20 data pin
const int irSensorPinL = 2;
const int irSensorPinR = 3;

// ========= Temperature Sensor =========
OneWire           oneWire(tempPin);
DallasTemperature sensors(&oneWire);

// ========= Analog Inputs =========
const int potRear[3]  = {A0, A1, A2};
const int potFront[3] = {A3, A4, A5};
const int potBlindL   = A6;
const int potBlindR   = A7;

// ========= Calibrate your ADC here =========
const int   ADC_MAX_COUNTS = 1023;                        
const float DIST_SCALE     = 600.0 / ADC_MAX_COUNTS;    

// ========= Sensor & State Variables =========
const int MAX_SENSORS = 6;
int distances[MAX_SENSORS] = {0};
int blindLeft  = 999;
int blindRight = 999;
int tempC      = 0;

int  sensorCount = 2;     // 2 or 3, set by jumper
int  irLeft  = 0;
int  irRight = 0;

// ========= Streaming flags – permanently ON =========
const bool rearStreaming  = true;
const bool frontStreaming = true;

String btBuffer = "";      // used for DEBUG:1/0
unsigned long lastSensorTime   = 0;
const unsigned long sensorInterval = 100;   // ms

// ===================================================================
void setup() {
  Serial.begin(38400);
  bluetooth.begin(38400);

  pinMode(debugPin,      INPUT_PULLUP);
  debugEnabled = (digitalRead(debugPin) == LOW);
  if (debugEnabled) Serial.println(F("Debug Mode ENABLED"));

  pinMode(modePin,       INPUT_PULLUP);
  pinMode(irSensorPinL,  INPUT_PULLUP);
  pinMode(irSensorPinR,  INPUT_PULLUP);

  sensors.begin();

  debugPrintln(F("Sensor unit ready streaming starts now."));  
}

void loop() {
  handleDebugCommands();     // only DEBUG:1 / DEBUG:0
  jumperCheck();

  if (millis() - lastSensorTime >= sensorInterval) {
    readPotDistances();
    readPotBlindSpots();
    readIRButtons();
    readTemperature();
    sendDataPacket();
    lastSensorTime = millis();
  }
}

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

void jumperCheck() {
  sensorCount = (digitalRead(modePin) == LOW) ? 2 : 3;
}

// ========= Analog → Parking distances =========
void readPotDistances() {
  for (int i = 0; i < sensorCount; i++) {
    distances[i]               = min((int)(analogRead(potRear[i])  * DIST_SCALE + 0.5), 460);
    distances[i + sensorCount] = min((int)(analogRead(potFront[i]) * DIST_SCALE + 0.5), 460);
  }
}

// ========= Analog → Blind‑spot distances =========
void readPotBlindSpots() {
  int rawLeft  = analogRead(potBlindL);
  int rawRight = analogRead(potBlindR);

  blindLeft  = max(0, (int)((ADC_MAX_COUNTS - rawLeft)  * DIST_SCALE + 0.5));
  blindRight = max(0, (int)((ADC_MAX_COUNTS - rawRight) * DIST_SCALE + 0.5));

  blindLeft  = min(blindLeft,  600);  // cap to 200 cm if you want
  blindRight = min(blindRight, 600);
}


// ========= IR line‑sensor inputs =========
void readIRButtons() {
  irLeft  = (digitalRead(irSensorPinL) == LOW) ? 1 : 0;
  irRight = (digitalRead(irSensorPinR) == LOW) ? 1 : 0;
}

// ========= Temperature =========
void readTemperature() {
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0);
}

// ========= Bluetooth packet send =========
void sendDataPacket() {
  bool tempError = (tempC == DEVICE_DISCONNECTED_C);

  uint8_t buf[32];
  uint8_t idx = 0;

  buf[idx++] = 0xAA;                        // HEADER
  buf[idx++] = 0x02;                        // TYPE
  buf[idx++] = 0x03;                        // FLAGS: rear + front always 1

  // ========= Rear distances =========
  for (int i = 0; i < 3; i++) {
    uint16_t val = (i < sensorCount) ? distances[i] : 0xFFFF;
    buf[idx++] = val & 0xFF;
    buf[idx++] = val >> 8;
  }

  // ========= Front distances =========
  for (int i = 0; i < 3; i++) {
    uint16_t val = (i < sensorCount) ? distances[i + sensorCount] : 0xFFFF;
    buf[idx++] = val & 0xFF;
    buf[idx++] = val >> 8;
  }

  // ========= Blind‑spot distances =========
  buf[idx++] = blindLeft  & 0xFF;  buf[idx++] = blindLeft  >> 8;
  buf[idx++] = blindRight & 0xFF;  buf[idx++] = blindRight >> 8;

  // ========= Temperature =========
  uint16_t t = tempError ? 0xFFFF : (uint16_t)tempC;
  buf[idx++] = t & 0xFF;  buf[idx++] = t >> 8;

  // ========= IR line flags =========
  buf[idx++] = (irLeft ? 1 : 0) | (irRight ? 2 : 0);

  buf[idx++] = 0x55;                       // TAIL

  bluetooth.write(buf, idx);

  if (debugEnabled) {
    Serial.print(F("Sent: "));
    for (uint8_t i = 0; i < idx; i++) { Serial.print(buf[i], HEX); Serial.print(' '); }
    Serial.println();
  }
}
