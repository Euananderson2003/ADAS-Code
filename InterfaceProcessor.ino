#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// === Debug Pin Setup ===
const int debugPin = 12;
bool debugEnabled = false;

#define debugPrint(x)   if (debugEnabled) Serial.print(x)
#define debugPrintln(x) if (debugEnabled) Serial.println(x)

// === LCD & Bluetooth Setup ===
LiquidCrystal_I2C lcd(0x27, 20, 4);
SoftwareSerial bluetooth(10, 11); // RX, TX

// === Pins (LEDs, Buzzer) ===
const int tempLED      = 2;
const int buzzerPin    = 3;
const int blindLeftLED = 6;
const int blindRightLED= 7;
const int irLeftLED    = 8;
const int irRightLED   = 9;

// === Sensor Data ===
const int MAX_SENSORS     = 6;
const int INACTIVE_READING= 999;
const int CLEAR_FLAG      = 666;
int  distances[MAX_SENSORS] = {INACTIVE_READING};
int  tempC        = 0;
int  sensorCount  = 0;
int  irLeft       = -1;
int  irRight      = -1;
int  blindLeft    = INACTIVE_READING;
int  blindRight   = INACTIVE_READING;

// === State Flags (permanently ON) ===
bool rearModeOn  = true;  // <‑‑ always active
bool frontModeOn = true;  // <‑‑ always active

// === Bluetooth Connection State ===
bool           btConnected   = false;
unsigned long  lastBtMsgTime = 0;
const unsigned long btTimeout = 5000UL;

// === Timing Control ===
unsigned long lastLcdUpdate = 0;
const unsigned long lcdInterval = 100;

// === Temp ===
bool tempReady = false;

// === LCD State Tracking ===
int lastDisplayedTemp = -999;

// ===================================================================
void setup() {
  Serial.begin(38400);
  bluetooth.begin(38400);

  pinMode(debugPin, INPUT_PULLUP);
  debugEnabled = (digitalRead(debugPin) == LOW);
  if (debugEnabled) Serial.println("Debug Mode ENABLED");

  // LCD splash =======================================================
  lcd.begin(20, 4);
  lcd.backlight();
  lcd.setCursor(0, 1); lcd.print("  Advanced Driver");
  lcd.setCursor(0, 2); lcd.print(" Assistance System");
  delay(2000);
  lcd.clear();
  lcd.setCursor(0, 1); lcd.print("    Waiting for");
  lcd.setCursor(0, 2); lcd.print("    Bluetooth...");
  lastDisplayedTemp = -999;

  // I/O ==============================================================
  pinMode(tempLED,       OUTPUT);
  pinMode(buzzerPin,     OUTPUT);
  pinMode(blindLeftLED,  OUTPUT);
  pinMode(blindRightLED, OUTPUT);
  pinMode(irLeftLED,     OUTPUT);
  pinMode(irRightLED,    OUTPUT);

  debugPrintln("System ready. Rear & Front modes are permanently ON.");
}

// ===================================================================
void loop() {
  // Handle Bluetooth timeout ========================================
  if (btConnected && (millis() - lastBtMsgTime > btTimeout)) {
    btConnected = false;
    lcd.clear();
    lcd.setCursor(0, 1); lcd.print("     Bluetooth ");
    lcd.setCursor(0, 2); lcd.print("    Disconnected");
    delay(5000);
    lcd.clear();
    lcd.setCursor(0, 1); lcd.print("    Waiting for");
    lcd.setCursor(0, 2); lcd.print("    Bluetooth...");
    lastDisplayedTemp = -999;
    debugPrintln("Bluetooth disconnected");
  }

  messageDecode();   // parse incoming sensor frame

  if (btConnected) {
    LEDs();
    checkProximityAlert();
  } else {
    noTone(buzzerPin);
    digitalWrite(tempLED, LOW);
    digitalWrite(blindLeftLED,  LOW);
    digitalWrite(blindRightLED, LOW);
    digitalWrite(irLeftLED,     LOW);
    digitalWrite(irRightLED,    LOW);
  }

  if (millis() - lastLcdUpdate >= lcdInterval) {
    LCDPrint();
    lastLcdUpdate = millis();
  }
}

// =======================  Bluetooth Rx  ============================
void messageDecode() {
  static uint8_t buffer[32];
  static uint8_t index = 0;
  static bool    inFrame = false;

  while (bluetooth.available()) {
    uint8_t b = bluetooth.read();

    if (!inFrame) {
      if (b == 0xAA) {
        inFrame = true;
        index = 0;
        buffer[index++] = b;
      }
    } else {
      buffer[index++] = b;

      if (b == 0x55) {
        parseMessage(buffer, index);
        inFrame = false;
      } else if (index >= sizeof(buffer)) {
        debugPrintln("Frame overflow discarded.");
        inFrame = false;
      }
    }
  }
}

void parseMessage(uint8_t* msg, uint8_t len) {
  if (len < 20 || msg[0] != 0xAA || msg[len - 1] != 0x55) {
    debugPrintln("Invalid frame");
    return;
  }

  if (!btConnected) {
    btConnected = true;
    lcd.clear();
    lastDisplayedTemp = -999;
    debugPrintln("Bluetooth connected");

  }
  lastBtMsgTime = millis();

  uint8_t flags = msg[2];
  sensorCount = 3;

  for (int i = 0; i < 3; i++) {
    distances[i] = (flags & 0x01) ? (msg[3 + i * 2] | (msg[4 + i * 2] << 8)) : INACTIVE_READING;
  }
  for (int i = 0; i < 3; i++) {
    distances[3 + i] = (flags & 0x02) ? (msg[9 + i * 2] | (msg[10 + i * 2] << 8)) : INACTIVE_READING;
  }

  blindLeft  = msg[15] | (msg[16] << 8);
  blindRight = msg[17] | (msg[18] << 8);

  int tempRaw = msg[19] | (msg[20] << 8);
  if (tempRaw != 0xFFFF && tempRaw > -100 && tempRaw < 100) {
    tempC = tempRaw;
    tempReady = true;
  }


  irLeft  = (msg[21] & 0x01) ? 1 : 0;
  irRight = (msg[21] & 0x02) ? 1 : 0;

  // Debug dump ===================================================
  debugPrint("RX: ");
  for (int i = 0; i < len; i++) { Serial.print(msg[i], HEX); Serial.print(" "); }
  Serial.println();
}

// ===================================================================
void LCDPrint() {
  if (!btConnected) return;

  int minRear  = INACTIVE_READING;
  int minFront = INACTIVE_READING;

  // Rear minimum ===================================================
  for (int i = 0; i < sensorCount && i < 3; i++) {
    if (distances[i] >= 0 && distances[i] < minRear)
      minRear = distances[i];
  }

  // Front minimum ===================================================
  for (int i = 0; i < sensorCount && i < 3; i++) {
    int fIndex = i + sensorCount;
    if (fIndex < MAX_SENSORS && distances[fIndex] >= 0 && distances[fIndex] < minFront)
      minFront = distances[fIndex];
  }

  // Display =========================================================
lcd.setCursor(0, 0);
lcd.print("Rear: ");
lcd.print((minRear == INACTIVE_READING) ? "---     " :
          (minRear >= 450)              ? "Clear    " :
          String(minRear) + " cm   ");

lcd.setCursor(0, 1);
lcd.print("Front: ");
lcd.print((minFront == INACTIVE_READING) ? "---     " :
          (minFront >= 450)              ? "Clear    " :
          String(minFront) + " cm   ");


  // Temperature ===================================================
lcd.setCursor(16, 0);
if (tempReady && tempC != lastDisplayedTemp && tempC < INACTIVE_READING) {
  lcd.print("     ");                      // clear full field (5 spaces)
  lcd.setCursor(16, 0);
  if (tempC < 10) lcd.print(" ");          // leading space for alignment
  lcd.print(tempC); lcd.print((char)223); lcd.print("C");
  lastDisplayedTemp = tempC;
} else if (!tempReady || tempC >= INACTIVE_READING) {
  lcd.setCursor(16, 0);
  lcd.print("ERR  ");                      // overwrite fully
  lastDisplayedTemp = -999;
}


  // Flags ==========================================================
  lcd.setCursor(0, 2); lcd.print("BSM L:"); lcd.print((blindLeft  > 0 && blindLeft  < 600) ? "Yes" : "No ");
  lcd.setCursor(9, 2); lcd.print("  R:");   lcd.print((blindRight > 0 && blindRight < 600) ? "Yes" : "No ");
  lcd.setCursor(0,3); lcd.print("WLM L:"); lcd.print(irLeft  == 1 ? "Yes " : "No  ");
  lcd.setCursor(9,3); lcd.print("  R:");     lcd.print(irRight == 1 ? "Yes " : "No ");
}

// ===================================================================
void LEDs() {
  digitalWrite(tempLED,       (tempC <= 3 && tempC > -100 && tempReady) ? HIGH : LOW);
  digitalWrite(blindLeftLED,  (blindLeft  > 0 && blindLeft  < 600)      ? HIGH : LOW);
  digitalWrite(blindRightLED, (blindRight > 0 && blindRight < 600)      ? HIGH : LOW);
  digitalWrite(irLeftLED,     (irLeft  == 1) ? HIGH : LOW);
  digitalWrite(irRightLED,    (irRight == 1) ? HIGH : LOW);
}

// ===================================================================
void checkProximityAlert() {
  static unsigned long lastBeepTime = 0;
  static bool          beepOn       = false;

  int minDistance = INACTIVE_READING;
  for (int i = 0; i < sensorCount * 2 && i < MAX_SENSORS; i++) {
    if (distances[i] >= 0 && distances[i] < minDistance)
      minDistance = distances[i];
  }

  unsigned long now = millis();
  int frequency = 0;
  unsigned long onTime = 0, offTime = 0;

  if (minDistance < 20)       { frequency = 4000; onTime = 100; offTime =  50; }
  else if (minDistance < 35)  { frequency = 4000; onTime = 100; offTime = 100; }
  else if (minDistance < 50)  { frequency = 4000; onTime = 100; offTime = 150; }
  else { noTone(buzzerPin); beepOn = false; return; }

  if (!beepOn && now - lastBeepTime >= offTime) {
    tone(buzzerPin, frequency);
    beepOn = true;
    lastBeepTime = now;
  } else if (beepOn && now - lastBeepTime >= onTime) {
    noTone(buzzerPin);
    beepOn = false;
    lastBeepTime = now;
  }
}
