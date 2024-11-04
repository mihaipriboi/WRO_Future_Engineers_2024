// -----Servo-----
#define SERVO_PIN D2
#define ANGLE_MIN 7 // 10
#define ANGLE_MID 47 // 50
#define ANGLE_MAX 87 // 90
#define ANGLE_VARIANCE_THRESHOLD (ANGLE_MAX * 0.4)
#define STEP 4
Servo servo;
double goal_deg;

// -----Motor_Drive-----
#define PWMA A2
#define AIN2 A1
#define AIN1 A0
#define DRIVER_PWM_CHANNEL 1
#define PWM_FREQ 5000 // 5 kHz
#define PWM_RES 10 // 10-bit resolution

// -----Encoder-----
#define ENCODER_PIN1 D5
#define ENCODER_PIN2 D4
#define GEAR_RATIO 1. / 50
#define WHEEL_DIAM 49.5
Encoder myEnc(ENCODER_PIN1, ENCODER_PIN2);

void motor_driver_setup() {
  ledcSetup(DRIVER_PWM_CHANNEL, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, DRIVER_PWM_CHANNEL);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
}

void servo_setup() {
  servo.attach(SERVO_PIN);
  for (int deg = servo.read() - 1; deg >= ANGLE_MIN; deg--)
    servo.write(deg);
  delay(500);
  // Serial.println("after ANGLE_MIN");
  for (int deg = servo.read() + 1; deg <= ANGLE_MID; deg++)
    servo.write(deg);
  // Serial.println("after ANGLE_MID");
  for (int deg = servo.read() + 1; deg <= ANGLE_MAX; deg++)
    servo.write(deg);
  delay(500);
  // Serial.println("after ANGLE_MAX");
  for (int deg = servo.read() - 1; deg >= ANGLE_MID; deg--)
    servo.write(deg);
  delay(500);
  // Serial.println("after ANGLE_MID");
  goal_deg = ANGLE_MID;
}

void move_motor(double speed) {  // move the motor with a given speed in the [0, 100] interval
  int dir = 1;
  if (speed < 0) {
    dir = -1;
    speed *= -1;
  }
  else if (speed == 0) {
    dir = 0;
  }
  speed = map_double(speed, 0, 100, 0, 1023);
  if (dir == 1) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  else if (dir == -1) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
  }
  ledcWrite(DRIVER_PWM_CHANNEL, (int)speed);
}

void custom_delay(long long delay_time) {
  long long start_time = millis();
  while (millis() - start_time < delay_time) {
    read_gyro(false);
    while (Serial0.available() > 0) { // if we have some characters waiting
      Serial0.read(); // flush the character
    }
  }
}

void motor_break(long long break_time) {
  move_motor(-3);
  custom_delay(break_time);
}

double read_motor_cm() {  // getting the distance driven by the motor in cm
  return GEAR_RATIO * WHEEL_DIAM * M_PI * (double)myEnc.read() / 12 / 10;
}

void move_servo(double angle) {  // move the servo to the angle checkpoint by setting the goal degrees to the angle value
  goal_deg = map_double(angle, -1, 1, ANGLE_MIN, ANGLE_MAX);
}

void update_servo() {
  int current_angle_servo = servo.read();
  // print_angles();
  if (abs(current_angle_servo - goal_deg) >= ANGLE_VARIANCE_THRESHOLD) {
    servo.write(goal_deg);
  }
  else {
    if (current_angle_servo < goal_deg) {
      servo.write(min(current_angle_servo + STEP, ANGLE_MAX));
    }
    else if (current_angle_servo > goal_deg) {
      servo.write(max(current_angle_servo - STEP, ANGLE_MIN));
    }
  }
}

void move_until_angle(double speed, double gyro_offset) {
  int sign = 1;
  if (speed < 0)
    sign = -1;
  read_gyro(false);
  double err = gyro_offset - gx;
  while (abs(err) >= 10) {
    read_gyro(false);
    err = gyro_offset - gx;
    pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
    pid_last_error_gyro = pid_error_gyro;
    move_servo(pid_error_gyro * sign);
    update_servo();
    move_motor(speed);
    while (Serial0.available() > 0) { // if we have some characters waiting
      Serial0.read(); // flush the character
    }
  }
}

void move_cm_gyro(double dis, double speed, double gyro_offset) {
  double start_cm = read_motor_cm();
  int sign = 1;
  if (speed < 0)
    sign = -1;
  while (abs(read_motor_cm() - start_cm) < dis) {
    read_gyro(false);
    double err = gyro_offset - gx;
    pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
    pid_last_error_gyro = pid_error_gyro;
    move_servo(pid_error_gyro * sign);
    update_servo();
    move_motor(speed);
    while (Serial0.available() > 0) { // if we have some characters waiting
      Serial0.read(); // flush the character
    }
  }
}

void print_angles() {  // for debugging reasons
  int current_angle = servo.read();
  Serial.print("goal: ");
  Serial.print(goal_deg);
  Serial.print("\tcurrent: ");
  Serial.println(current_angle);
}