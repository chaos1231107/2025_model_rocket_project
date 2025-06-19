#include "arduinoFFT.h"

#define SAMPLES 64
#define SAMPLING_FREQUENCY 100

ArduinoFFT<double> FFT = ArduinoFFT<double>();

double vReal[SAMPLES];
double vImag[SAMPLES];

unsigned long sampling_period_us;
unsigned long microseconds;

void setup() {
  Serial.begin(115200);
  sampling_period_us = 1000000 / SAMPLING_FREQUENCY;
}

void loop() {
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();

    vReal[i] = analogRead(A0);
    vImag[i] = 0;

    while (micros() - microseconds < sampling_period_us) {
      // wait for next sample time
    }
  }

  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  Serial.println("Frequency (Hz), Magnitude");
  for (int i = 0; i < (SAMPLES / 2); i++) {
    double freq = (i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES;
    Serial.print(freq);
    Serial.print(", ");
    Serial.println(vReal[i]);
  }

  delay(1000);
}
