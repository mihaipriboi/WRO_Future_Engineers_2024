String receivedMessage;

void blink_led(int pin, int duration) {
  digitalWrite(pin, HIGH);
  custom_delay(duration);
  digitalWrite(pin, LOW);
  custom_delay(duration);
}

void setup_led(int pin) {
  pinMode(pin, OUTPUT);
}

void setup_periferics() {
  setup_led(LED_BUILTIN);
  setup_led(DEBUG_LED);
  pinMode(BTN_PIN, INPUT_PULLUP);
}

void comm_setup() {
  Serial.begin(9600);
  // while(!Serial);
  blink_led(LED_BUILTIN, 500);

  Serial0.begin(19200);
  while(!Serial0); // wait for the serial to properly initialize
  blink_led(LED_BUILTIN, 500);
  receivedMessage = "";
}

void setup_function() {
  // put your setup code here, to run once:
  setup_periferics();

  comm_setup();

  motor_driver_setup(); // must be motor driver setup first and then servo setup
  servo_setup();

  if (FINAL)
    blink_led(LED_BUILTIN, 500);
  else
    blink_led(LED_BUILTIN, 100);
  gyro_setup(true);

  // turn on debugging leds so that we know the setup is complete
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(DEBUG_LED, HIGH);
}