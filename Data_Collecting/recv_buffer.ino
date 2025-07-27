#include <AltSoftSerial.h>
#include <SD.h>
#include <SPI.h>

AltSoftSerial hc12;
const int cs = 10;
String recv = "";
unsigned long lastReceivedTime = 0;
// packet terminate time
const unsigned long timeout = 150;

void process_message(const String &msg) {
  Serial.print("From Pi: ");
  Serial.println(msg);
  delay(10);
  //hc12.println(msg);
}

void setup() {
  Serial.begin(9600);
  hc12.begin(19200);
  Serial.println("Arduino ready and listening and Initializing SD Card...");

  if (!SD.begin(cs)) {
    Serial.print("SD Card initializing Faild!");
  }

}

const int max_buffer = 256;
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

      else if (c >= 32 && c <= 256) {
        recv += c;
      }
  }

  if (recv.length() > 0 && millis() - lastReceivedTime > timeout) {
    Serial.println("Packet Lost Occured...Initializing Recieve Buffer");
    process_message(recv);
    
    recv = "";
  }

}
