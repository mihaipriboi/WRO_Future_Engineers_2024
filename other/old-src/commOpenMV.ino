#include <math.h>
#include <Encoder.h>

// -----Motor_Drive-----
const int IN1 = 15;
const int IN2 = 14;
const int EEP = 12;

// -----Encoder-----
const int ENCODER_PIN1 = D4;
const int ENCODER_PIN2 = D5;
const double GEAR_RATIO = 1./50;
const double WHEEL_DIAM = 49.5;

Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial0.begin(19200);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EEP, OUTPUT);
  digitalWrite(EEP, HIGH);
}

String receivedMessage;

void execute(String cmd) {
  int pos = 0;
  String motor = "";
  while (cmd[pos] != ' ') {
    motor += cmd[pos];
    pos++;
  }
  pos++;
  int speed = 0, sign = 1;
  if (cmd[pos] == '-')
    sign = -1, pos++;
  else if (cmd[pos] == '+')
    sign = 1, pos++;
  while ('0' <= cmd[pos] && cmd[pos] <= '9') {
    speed = speed * 10 + cmd[pos] - '0';
    pos++;
  }
  speed *= sign;
  moveMotor(speed);
}

inline void moveMotor(int speed) {
  int dir = 1;
  if (speed < 0) {
    dir = -1;
    speed *= -1;
  }
  if (speed > 100)
    speed = 100;
  speed = map(speed, 0, 100, 0, 255);
  if (dir == 1) {
    analogWrite(IN1, speed);
    digitalWrite(IN2, LOW);
  }
  else {
    digitalWrite(IN1, LOW);
    analogWrite(IN2, speed);
  }
}

double read_motor_cm(double wheel_diameter, double gear_ratio) {  // getting the distance driven by the motor in cm
  return gear_ratio * wheel_diameter * M_PI * (double)myEnc.read() / 12 / 10;
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial0.available() > 0) { // if we have some characters waiting
    char receivedChar = Serial0.read(); // we get the first character
    if (receivedChar == '\n') { // if it's the end of message marker
      // Serial.println(receivedMessage);  // Print the received message in the Serial monitor
      execute(receivedMessage); // execute the received command from the OpenMV camera
      Serial.println(read_motor_cm(WHEEL_DIAM, GEAR_RATIO)); // get and print the distanced
      receivedMessage = "";  // Reset the received message
    } else {
      receivedMessage += receivedChar;  // Append characters to the received message
    }
  }
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}