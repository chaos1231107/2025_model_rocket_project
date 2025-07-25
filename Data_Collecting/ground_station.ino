#include <AltSoftSerial.h>
#include <SPI.h>
#include <SD.h>
AltSoftSerial hc12;

const int cs = 10;

void setup() {
  Serial.begin(9600);
  hc12.begin(19200);

  Serial.println("Initializing SD Card...");

  if (!SD.begin(cs)) {
    Serial.print("SD Card initilization Faild!");
    //Serial.println("Arduino ready and listening... Type 'd' in Serial Monitor if you want force ejection");
  }

}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      hc12.println(input);
      Serial.println("Sent via HC-12: " + input);
    }
  }
  if (hc12.available()) {
    String msg = hc12.readStringUntil('\n');

    msg.trim();

    Serial.print("From Pi: ");
    Serial.println(msg);

    File log = SD.open("log.txt", FILE_WRITE);
    if (log) {
      log.println(msg);
      log.close();
      Serial.println("Data saved to SD Card");
    }

    else {
      Serial.println("Failed to open log.txt");
    }
  }

}
