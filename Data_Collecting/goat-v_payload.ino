#include <AltSoftSerial.h>
#include <SD.h>
#include <SPI.h>

AltSoftSerial hc12;
const int CS = 10;

// ---- 프로토콜/버퍼 ----
static const uint16_t TIMEOUT_MS = 250;   // 부분 프레임 유효시간
static const size_t   MAX_PAYLOAD = 64;   
char   recv_buf[MAX_PAYLOAD + 1];
size_t recv_len = 0;
bool   in_frame = false;
unsigned long last_rx_ms = 0;

// ---- 유틸 ----
inline bool is_printable(uint8_t c) {
  return (c >= 32 && c <= 126); // ASCIII 가독 문자
}

void log_to_sd(const char* msg, unsigned long now_ms) {
  File log = SD.open("log.txt", FILE_WRITE);
  if (log) {
    log.print("["); log.print(now_ms); log.print("ms] ");
    log.println(msg);
    log.close();
    Serial.println("Data Saved to SD Card");
  } else {
    Serial.println("Failed to open log.txt");
  }
}

void process_message(const char* msg) {
  const unsigned long now = millis();
  Serial.println(msg);
  log_to_sd(msg, now);
}

void reset_frame() {
  in_frame = false;
  recv_len = 0;
  recv_buf[0] = '\0';
}

void setup() {
  Serial.begin(9600);
  hc12.begin(19200);
  Serial.println("Arduino ready to listen and Initializing SD Card...");

  if (!SD.begin(CS)) {
    Serial.println("SD Card initializing Failed!");
  } else {
    Serial.println("SD Card Initialized.");
  }
}

void loop() {
  // 지상국 입력 → HC-12로 패스스루
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("Sent via Ground Station: ");
    Serial.println(cmd);
    hc12.write(cmd);
  }

  // ---- 수신 상태기계 ----
  while (hc12.available()) {
    uint8_t c = hc12.read();
    last_rx_ms = millis();

    if (!in_frame) {
      // 프레임 시작 대기
      if (c == '<') {
        in_frame = true;
        recv_len = 0;
      }
      // 그 외 문자는 무시(노이즈 컷)
      continue;
    }

    // in_frame == true
    if (c == '>') {
      // 프레임 종료 → 메시지 확정
      recv_buf[recv_len] = '\0';
      process_message(recv_buf);
      reset_frame();
      continue;
    }

    // 유효 페이로드만 수집
    if (is_printable(c)) {
      if (recv_len < MAX_PAYLOAD) {
        recv_buf[recv_len++] = (char)c;
      } else {
        // 오버플로우 → 프레임 폐기 및 동기 재획득
        Serial.println("Error: payload overflow, frame dropped");
        reset_frame();
      }
    }
    // 비가독 문자면 그냥 무시(동기 유지)
  }

  // ---- 부분 프레임 타임아웃 처리 ----
  if (in_frame && (millis() - last_rx_ms > TIMEOUT_MS)) {
    // 불완전 프레임은 버리는 게 맞음 (로그 오염 방지)
    Serial.println("Timeout: partial frame dropped");
    reset_frame();
  }
}
