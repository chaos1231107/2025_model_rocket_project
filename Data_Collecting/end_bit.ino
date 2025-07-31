#include <AltSoftSerial.h>
#include <SD.h>
#include <SPI.h>

AltSoftSerial hc12;
const int cs = 10;
String recv = "";
unsigned long lastReceivedTime = 0;
// packet terminate time
const unsigned long timeout = 120;

void process_message(const String &msg) {
  Serial.print("From Pi: ");
  Serial.println(msg);
  delay(30);
  //hc12.println(msg);
}

void setup() {
  Serial.begin(9600);
  hc12.begin(9600);
  Serial.println("Arduino ready to listen and Initializing SD Card...");

  if (!SD.begin(cs)) {
    Serial.print("SD Card initializing Faild!");
  }

}

const int max_buffer = 64;
void loop() {

  if (Serial.available()) {
    //String input = Serial.readStringUntil('\n');
    //input.trim();
    char cmd = Serial.read();
    Serial.print("sent via Ground Station: ");
    Serial.println(cmd);
    hc12.print(cmd);
  }

  while (hc12.available()) {
    char c = hc12.read();
    lastReceivedTime = millis();

    if (recv.length() > max_buffer) {
      recv = "";
      Serial.println("Error: Message too long, buffer reset");
      continue;
    }

    if (c == '\n' || c == '\r') {
      recv.trim();
      if (recv.length() > 0) {
        process_message(recv);
        File log = SD.open("log.txt", FILE_WRITE);

        if (log) {
          log.println(recv);
          log.close();
          Serial.println("Data Saved to SD Card");
        }

        else {
          Serial.println("Failed to open log.txt");
        }

        recv = "";
      }
    }

      else if (c >= 32 && c <= 126) {
        recv += c;
      }
  }
  bool timeout_cond = recv.length() > 0 && (millis() - lastReceivedTime > timeout);
  bool end_bit_cond = !recv.endsWith(">>") && !recv.startsWith("<<");

  if (timeout_cond && end_bit_cond) {
    Serial.println("Packet Loss Occured...Initializing Recieve Buffer");
    recv.trim();
    process_message(recv);
    
    recv = "";
  }

}
