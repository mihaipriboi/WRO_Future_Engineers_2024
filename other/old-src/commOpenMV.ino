#include <math.h>
#include <Encoder.h>
#include <ESP32Servo.h>

// -----Servo-----
const int SERVO_PIN = D2;
Servo servo;

// -----Motor_Drive-----
const int PWMA = A0;
const int AIN2 = A1;
const int AIN1 = A2;

// -----Encoder-----
const int ENCODER_PIN1 = D4;
const int ENCODER_PIN2 = D5;
const double GEAR_RATIO = 1. / 50;
const double WHEEL_DIAM = 49.5;
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

void comm_setup() {
  Serial0.begin(19200);
}

void motor_driver_setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void servo_setup() {
  servo.attach(SERVO_PIN, 500, 2400);
  move_servo(0);
  delay(50);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);

  comm_setup();
  motor_driver_setup();
  servo_setup();
}

String receivedMessage;

void execute(String cmd) {
  int pos = 0, type = 0;
  String motor = "";
  if (cmd[pos] == 's')
    type = 1, pos++;
  double val = (cmd + pos).toDouble();
  if (type == 0)
    move_motor((int)val);
  else
    move_servo(val);
}

inline void move_motor(int speed) {
  int dir = 1;
  if (speed < 0) {
    dir = -1;
    speed *= -1;
  }
  if (speed > 100)
    speed = 100;
  speed = map(speed, 0, 100, 0, 255);
  if (dir == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  analogWrite(PWMA, speed);
}

double read_motor_cm(double wheel_diameter, double gear_ratio) {  // getting the distance driven by the motor in cm
  return gear_ratio * wheel_diameter * M_PI * (double)myEnc.read() / 12 / 10;
}

void move_servo(double angle) {
  if (angle < -1)
    angle = -1;
  else if (angle > 1)
    angle = 1;
  angle = map(angle, -1, 1, 0, 180);
  servo.write(angle);
}

void loop() {
  // put your main code here, to run repeatedly:
  while (Serial0.available() > 0) { // if we have some characters waiting
    char receivedChar = Serial0.read(); // we get the first character
    if (receivedChar == '\n') { // if it's the end of message marker
      execute(receivedMessage); // execute the received command from the OpenMV camera
      Serial.println(receivedMessage);
      // Serial.println(read_motor_cm(WHEEL_DIAM, GEAR_RATIO)); // get and print the distanced
      receivedMessage = "";  // Reset the received message
    } else {
      receivedMessage += receivedChar;  // Append characters to the received message
    }
  }
}