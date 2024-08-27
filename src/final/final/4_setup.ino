String receivedMessage;

void blink_led(int pin, int duration) {
  digitalWrite(pin, HIGH);
  delay(duration);
  digitalWrite(pin, LOW);
  delay(duration);
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
  Serial0.begin(19200);
  while(!Serial0);
  blink_led(LED_BUILTIN, 500);
  receivedMessage = "";
}

void setup_function() {
  // put your setup code here, to run once:
  setup_periferics();

  Serial.begin(9600);
  // while(!Serial);

  motor_driver_setup(); // must be motor driver setup first and then servo setup
  servo_setup();

  comm_setup();
  gyro_setup(true);

  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(DEBUG_LED, HIGH);
}