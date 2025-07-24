#include <SPI.h>
#include <SD.h>

const int chipSelect = 10; // CS 핀 번호

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // 시리얼 포트가 연결될 때까지 대기
  }

  Serial.println("=== 간단한 SD카드 테스트 ===");

  // SD카드 초기화
  if (!SD.begin(chipSelect)) {
    Serial.println(" SD카드 초기화 실패!");
    Serial.println("확인사항:");
    Serial.println("- SD카드가 제대로 삽입되었는지");
    Serial.println("- 연결선이 올바른지");
    Serial.println("- CS핀이 10번인지");
    while (1);
  }
  
  Serial.println(" SD카드 초기화 성공!");

  // 테스트 파일 쓰기
  Serial.println("\n 파일 쓰기 테스트...");
  File testFile = SD.open("arduino.txt", FILE_WRITE);
  
  if (testFile) {
    testFile.println("Arduino SD Card Test");
    testFile.println("Time: " + String(millis()) + "ms");
    testFile.println("Status: Working!");
    testFile.close();
    Serial.println("파일 쓰기 성공!");
  } else {
    Serial.println("파일 쓰기 실패!");
  }

  // 테스트 파일 읽기
  Serial.println("\n 파일 읽기 테스트...");
  testFile = SD.open("arduino.txt");
  
  if (testFile) {
    Serial.println("파일 내용:");
    Serial.println("----------");
    while (testFile.available()) {
      Serial.write(testFile.read());
    }
    Serial.println("----------");
    testFile.close();
    Serial.println("파일 읽기 성공!");
  } else {
    Serial.println("파일 읽기 실패!");
  }

  // 파일 목록 출력
  Serial.println("\SD카드 파일 목록:");
  File root = SD.open("/");
  if (root) {
    File file = root.openNextFile();
    while (file) {
      Serial.print("AA ");
      Serial.print(file.name());
      if (!file.isDirectory()) {
        Serial.print(" (");
        Serial.print(file.size());
        Serial.print(" bytes)");
      }
      Serial.println();
      file.close();
      file = root.openNextFile();
    }
    root.close();
  }

  Serial.println("\n SD카드 테스트 완료!");
}

void loop() {
  // 1초마다 LED처럼 동작하는 표시
  Serial.print(".");
  delay(1000);
}
