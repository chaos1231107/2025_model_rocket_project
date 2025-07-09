#include <SoftwareSerial.h>

// HC-12 연결 핀: 아두이노 D2 = RX, D3 = TX
SoftwareSerial hc12(2, 3);  // (RX, TX)

void setup() {
  Serial.begin(9600);     // 시리얼 모니터 출력용
  hc12.begin(9600);       // HC-12 통신용
  Serial.println("Arduino ready and listening...");
}

void loop() {
  if (hc12.available()) {
    // HC-12에서 한 줄 단위 문자열 수신
    String msg = hc12.readStringUntil('\n');

    Serial.print("From Pi: ");
    Serial.println(msg);

    // 받은 문자열에 "Ack: " 붙여 다시 송신
    hc12.print("Ack: ");
    hc12.println(msg);
    //delay(500);
  }
}
