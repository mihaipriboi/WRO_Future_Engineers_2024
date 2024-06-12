#include <math.h>
#include <Encoder.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "BMI088.h"

// -----Button and Debug led-----
#define BTN_PIN A3
#define DEBUG_LED D12

void setup() {
  setup_function();
}

bool is_running = false;

void loop() {
  int is_start_btn_on = digitalRead(BTN_PIN);
  if (!is_running && is_start_btn_on == LOW) {
    is_running = true;
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_LED, LOW);
  }
  if (is_running) {
    loop_function();
  }
  // Serial.println("loop fr");
}