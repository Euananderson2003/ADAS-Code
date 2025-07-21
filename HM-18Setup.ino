=== Baudrate Calculator ===
#include <SoftwareSerial.h>

SoftwareSerial btSerial(10, 11); // RX, TX

long baudRates[] = {9600, 19200, 38400, 57600, 115200};
const int numRates = sizeof(baudRates) / sizeof(baudRates[0]);

void setup() {
  Serial.begin(9600);
  Serial.println("Starting Bluetooth baud rate test...");
}
void loop() {
  for (int i = 0; i < numRates; i++) {
    long currentBaud = baudRates[i];
    Serial.print("Testing baud rate: ");
    Serial.println(currentBaud);

    btSerial.begin(currentBaud);
    delay(100);

    // Send AT command
    btSerial.print("AT\r\n");
    delay(300);

    bool gotResponse = false;
    while (btSerial.available()) {
      char c = btSerial.read();
      Serial.write(c);
      gotResponse = true;
    }
    if (gotResponse) {
      Serial.print("Response at ");
      Serial.println(currentBaud);
      while (true); // Stop testing
    } else {
      Serial.println("No response.");
    }
    delay(1000);
  }
  Serial.println("Baud rate not found. Restarting test...");
  delay(3000);
}


// // === AT Command Configuration ===
// #include <SoftwareSerial.h>
// SoftwareSerial bt(10, 11); // RX, TX

// void setup() {
//   Serial.begin(115200);   // Match the HM-18’s current baud rate
//   bt.begin(115200);       // This must match HM-18’s known baud
//   Serial.println("Ready for AT commands...");
// }

// void loop() {
//   if (Serial.available()) bt.write(Serial.read());
//   if (bt.available()) Serial.write(bt.read());
// }



