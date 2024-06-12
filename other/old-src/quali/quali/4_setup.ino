String receivedMessage;

uint32_t last_rotate = 0, time_elapsed = 0;
int turns = 0, freq = 0;

void blink_led(int pin, int duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
  delay(duration);
}

void setup_led(int pin) {
  pinMode(pin, OUTPUT);
  blink_led(pin, 500);
}

void setup_periferics() {
  setup_led(LED_BUILTIN);
  setup_led(DEBUG_LED);
  pinMode(BTN_PIN, INPUT_PULLUP);
}

void comm_setup() {
  Serial0.begin(19200);
  receivedMessage = "";
}

void setup_function() {
  // put your setup code here, to run once:
  Serial.begin(2000000);

  setup_periferics();
  comm_setup();
  motor_driver_setup();
  servo_setup();
  gyro_setup(true);

  // time_elapsed = millis();
  // freq = 0;
  last_rotate = millis() - 1000;

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(DEBUG_LED, HIGH);
}