#include <SoftwareSerial.h>

SoftwareSerial hc12(2,3);  // HC-12 (RX, TX)

void setup() {
  //delay(100);
  Serial.begin(9600);
  //delay(1000);
  hc12.begin(9600);
  //delay(500);
  //Serial.println("Sending AT Command....");  
  //hc12.println("AT+B38400");   // HC-12 통신용
  Serial.println("Type 'DEPLOY' in Serial Monitor to send to Raspberry Pi");
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // 공백 제거

    if (input.length() > 0) {
      hc12.println(input);  // HC-12를 통해 라즈베리파이에 전송
      Serial.println("Sent via HC-12: " + input);
    }
  }

  // HC-12로부터 수신된 내용을 모니터에 출력 (필요시)
  if (hc12.available()) {
    String recv = hc12.readStringUntil('\n');
    recv.trim();
    Serial.println("Received from Raspberry Pi: " + recv);
  }
}
