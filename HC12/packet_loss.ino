#include <AltSoftSerial.h>

AltSoftSerial hc12;
String recv = "";
unsigned long lastReceivedTime = 0;
// packet terminate time
const unsigned long timeout = 80;

void process_message(const String &msg) {
  Serial.print("From Pi: ");
  Serial.println(msg);
  delay(20);
  hc12.println(msg);
}

void setup() {
  Serial.begin(9600);
  hc12.begin(19200);
  Serial.println("Arduino ready and listening...");

}

const int max_buffer = 256;
void loop() {
  while (hc12.available()) {
    char c = hc12.read();
    lastReceivedTime = millis();

    if (recv.length() > max_buffer) {
      recv = "";
      Serial.println("Error: Message too long, buffer reset");
      return;
    }

    if (c == '\n' || c == '\r') {
      recv.trim();
      if (recv.length() > 0) {
        process_message(recv);
        recv = "";
      }
    }

      else if (c >= 32 && c <= 256) {
        recv += c;
      }
  }

  if (recv.length() > 100 && millis() - lastReceivedTime > timeout) {
    Serial.println("Packet Loss Occured...Initializing Receive Buffer");
    recv.trim();
    process_message(recv);
    
    recv = "";
  }

}
