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

uint32_t last_rotate = 0, time_elapsed = 0;
int turns = 0, freq = 0;

void loop() {
  int is_start_btn_on = digitalRead(BTN_PIN);
  read_gyro(false);
  if (!is_running && is_start_btn_on == LOW) {
    is_running = true;
    
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_LED, LOW);
    
    last_rotate = millis();
    // time_elapsed = millis();
    // freq = 0;
  }
  if (is_running) {
    loop_function();
  }
}