#include <math.h>
#include <Encoder.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "BMI088.h"

#define FINAL true

// -----Button and Debug led-----
#define BTN_PIN A3
#define DEBUG_LED D12

void setup() {
  setup_function();
}

bool is_running = false;

uint32_t last_rotate = 0, time_elapsed = 0;

void loop() {
  int is_start_btn_on = digitalRead(BTN_PIN); // wait for the button to be pressed
  read_gyro(false); // flush the data
  flush_messages();
  if (!is_running && is_start_btn_on == LOW) { // if we press the button and the program isn't running
    is_running = true; // start the program
    
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(DEBUG_LED, LOW); // turn off the debugging leds

    custom_delay(3000);
    
    last_rotate = millis(); // initialize times
    // time_elapsed = millis();
    // freq = 0;
  }
  if (is_running) { // if the program is running, execute one iteration of the loop function
    loop_function();
  }
}