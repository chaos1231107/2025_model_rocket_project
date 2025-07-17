#include <AltSoftSerial.h>

AltSoftSerial hc12;  // RX = D8, TX = D9

void setup() {
  Serial.begin(9600);    // 시리얼 모니터
  hc12.begin(9600);      // HC-12는 아직 9600 상태
  Serial.println("AT command mode: Ready");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    hc12.write(c);
  }

  if (hc12.available()) {
    char c = hc12.read();
    Serial.write(c);
  }
}

