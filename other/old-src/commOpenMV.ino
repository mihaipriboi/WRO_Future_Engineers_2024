#include <math.h>
#include <Encoder.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include "BMI088.h"

// -----Button and Debug led-----
const int BTN_PIN = A7;
const int DEBUG_LED = A6;

// -----Servo-----
const int SERVO_PIN = D2;
Servo servo;
double goal_deg;
const int ANGLE_MID = 43;
const int ANGLE_LIMIT = ANGLE_MID * 2;
const double ANGLE_VARIANCE_THRESHOLD = ANGLE_LIMIT * 0.7;
const int STEP = 3;

// -----Motor_Drive-----
const int PWMA = A0;
const int AIN2 = A1;
const int AIN1 = A2;

// -----Encoder-----
const int ENCODER_PIN1 = D5;
const int ENCODER_PIN2 = D4;
const double GEAR_RATIO = 1. / 50;
const double WHEEL_DIAM = 49.5;
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

// -----Gyro-----
double gx, gy, gz;
long last_gyro_read;
long gyro_read_interval = 1;
bool gyro_flag, accel_flag;

String receivedMessage;

int freq, time_elapsed;

void comm_setup() {
  Serial0.begin(19200);
  receivedMessage = "";
}

void motor_driver_setup() {
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void servo_setup() {
  // servo.attach(SERVO_PIN, 400, 2400);
  servo.attach(SERVO_PIN);
  Serial.println("before 0");
  for (int deg = servo.read() - 1; deg >= 0; deg--)
    servo.write(deg);
  delay(500);
  Serial.println("after 0");
  for (int deg = servo.read() + 1; deg <= ANGLE_MID; deg++)
    servo.write(deg);
  delay(500);
  for (int deg = servo.read() + 1; deg <= ANGLE_LIMIT; deg++)
    servo.write(deg);
  delay(500);
  Serial.println("after ANGLE_LIMIT");
  for (int deg = servo.read() - 1; deg >= ANGLE_MID; deg--)
    servo.write(deg);
  Serial.println("after 0");
  goal_deg = ANGLE_LIMIT / 2;
}

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
  pinMode(BTN_PIN, INPUT);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  setup_periferics();
  comm_setup();
  servo_setup();
  motor_driver_setup();
  // gyro_setup(true);

  // freq = 0;
  // time_elapsed = millis();
}

void execute(String cmd) {
  int pos = 0, type = 0, sign = 1;
  String motor = "";
  if (cmd[pos] == 's')
    type = 1, pos++;
  // manually going over the signs since the .toDouble function wouldn't parse them on its own
  if (cmd[pos] == '+')
    sign = 1, pos++;
  else if (cmd[pos] == '-')
    sign = -1, pos++;
  double val = cmd.substring(pos).toDouble();
  if (type == 0)
    move_motor((int)val * sign);
  else
    move_servo(val * sign);
}

double clamp(double val, double left, double right) {  // force the val into the [left, right] interval
  if (val > right)
    return right;
  if (val < left)
    return left;
  return val;
}

double map_double(double x, double in_min, double in_max, double out_min, double out_max) {  // translate the x value from the [in_min, in_max] interval to the [out_min, out_max] interval
  const double run = in_max - in_min;
  if (run == 0)
    return out_min;
  x = clamp(x, in_min, in_max);
  const double rise = out_max - out_min;
  const double delta = x - in_min;
  return (delta * rise) / run + out_min;
}

inline void move_motor(double speed) {  // move the motor with a given speed in the [0, 100] interval
  int dir = 1;
  if (speed < 0) {
    dir = -1;
    speed *= -1;
  } else if (speed == 0)
    dir = 0;
  speed = map_double(speed, 0, 100, 0, 255);
  if (dir == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else if (dir == -1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  analogWrite(PWMA, (int)speed);
}

double read_motor_cm(double wheel_diameter, double gear_ratio) {  // getting the distance driven by the motor in cm
  return gear_ratio * wheel_diameter * M_PI * (double)myEnc.read() / 12 / 10;
}

void move_servo(double angle) {  // move the servo to the angle checkpoint by setting the goal degrees to the angle value
  goal_deg = map_double(angle, -1, 1, 0, ANGLE_LIMIT);
}

void print_angles() {  // for debugging reasons
  int current_angle = servo.read();
  Serial.print("goal: ");
  Serial.print(goal_deg);
  Serial.print("\tcurrent: ");
  Serial.println(current_angle);
}

// int deg_debug = 0, sign = 1;
// double delta_deg_debug = 0.1, offset_deg_debug = 0.1;

// void update_goal_deg_for_debug() {
//   if ( abs(deg_debug) > 1 ) {
//     deg_debug = 0;
//     offset_deg_debug = 0;
//   }
//   else {
//     deg_debug += offset_deg_debug * sign;
//     offset_deg_debug += delta_deg_debug;
//     sign *= -1;
//   }
// }

void loop() {
  // measure the processing frequency
  int t = millis() - time_elapsed;
  if (t > 1000) {
    time_elapsed = millis();
    // Serial.println(((double)freq / t) * 1000);
    // update_goal_deg_for_debug();
    // move_servo(deg_debug);

    freq = 0;
  }

  move_motor(100);
  // blink_led(DEBUG_LED, 1000);
  // if ( read_motor_cm(WHEEL_DIAM, GEAR_RATIO) >= 10 )
  //   digitalWrite(LED_BUILTIN, HIGH);
  // delay(1);

  // update the servo for steering for this loop
  int current_angle = servo.read();
  int angle_diff = abs(current_angle - goal_deg);
  // print_angles();
  if (angle_diff >= 0) {
    if (angle_diff >= ANGLE_VARIANCE_THRESHOLD)
      servo.write(goal_deg);
    else {
      if (current_angle < goal_deg) {
        servo.write(min(current_angle + STEP, ANGLE_LIMIT));
      } else if (current_angle > goal_deg) {
        servo.write(max(current_angle - STEP, STEP));
      }
    }
  }

  // // prioritize executing pending commands
  while (Serial0.available() > 0) { // if we have some characters waiting
    char receivedChar = Serial0.read(); // we get the first character
    if (receivedChar == '\n') { // if it's the end of message marker
      // Serial.println(receivedMessage);
      execute(receivedMessage); // execute the received command from the OpenMV camera
      receivedMessage = ""; // Reset the received message
    } else {
      receivedMessage += receivedChar; // Append characters to the received message
    }
  }

  for ( int i = 0; i < 100; i ++ ) // emulate some processing
    int r = random();

  // double init, cm;
  // cm = init = read_motor_cm(WHEEL_DIAM, GEAR_RATIO);
  // while ( cm - init < 2 ) {
  //   move_motor(70);
  //   cm = read_motor_cm(WHEEL_DIAM, GEAR_RATIO);
  // }
  // move_motor(0);

  // read_gyro(false);
  // if(millis() - last_gyro_read > 10) {
    
  //   Serial.print("gx: ");
  //   Serial.print(gx);
  //   Serial.print(" gy: ");
  //   Serial.print(gy);
  //   Serial.print(" gz: ");
  //   Serial.println(gz);

  //   last_gyro_read = millis();
  // }

  freq++;
}