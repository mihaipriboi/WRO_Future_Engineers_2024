// -----Cases-----
#define PID 0
#define STOP 1
#define FOLLOW_CUBE 2
#define AFTER_CUBE 3

// -----Velocities and distances-----
#define CORRECTION_ANGLE 50
#define AVOIDANCE_ANGLE 50
#define CORNER_DISTANCE_FINAL 10
#define CORNER_DISTANCE_QUALI 42 // maybe 40 if the battery is overcharged
#define MOTOR_SPEED 100

double avoid_cube_angle = 0;

int CASE = PID;
int turn_direction = 0;
int delay_walls = 250;
int cube_last = 0;

void pass_cube(int cube_last) {
  read_gyro(false);
  int start_angle = gx;
  move_until_angle(MOTOR_SPEED, start_angle + cube_last * -AVOIDANCE_ANGLE);
  move_cm_gyro(7, MOTOR_SPEED, start_angle + cube_last * -AVOIDANCE_ANGLE);
  CASE = AFTER_CUBE;
}

void execute(String cmd) {
  if (cmd[0] == 'R' || cmd[0] == 'G') {
    if (cmd[0] == 'R') {
      cube_last = 1;
    }
    else {
      cube_last = -1;
    }
    pass_cube(cube_last);
    return;
  }
  int pos = 0, sign = 1;
  String motor = "";
  if (cmd[pos] == 'r' || cmd[pos] == 'g')
    pos++;
  // manually going over the signs since the .toDouble function wouldn't parse them on its own
  if (cmd[pos] == '+')
    sign = 1, pos++;
  else if (cmd[pos] == '-')
    sign = -1, pos++;
  double val = cmd.substring(pos).toDouble();
  if (cmd[0] == 'r' || cmd[0] == 'g') {
    avoid_cube_angle = val * sign;
    CASE = FOLLOW_CUBE;
    return;
  }
  
  int msg = (int)val;
  if (msg) {
    if (msg == 1) { // blue line
      turn_direction = 1;
    }
    else { // orange line
      turn_direction = -1;
    }
    if (millis() - last_rotate > delay_walls) {
      if (abs(current_angle_gyro - gx) < 2.5) {
        move_cm_gyro(CORNER_DISTANCE_FINAL, MOTOR_SPEED, current_angle_gyro); // change according to the task CORNER_DISTANCE_FINAL or CORNER_DISTANCE_QUALI
      }
      else if (CASE != FOLLOW_CUBE) {
        move_until_angle(MOTOR_SPEED, current_angle_gyro + turn_direction * 20);
        motor_break(2000);
        move_cm_gyro(15, MOTOR_SPEED, current_angle_gyro); // change according to the task CORNER_DISTANCE_FINAL or CORNER_DISTANCE_QUALI
        motor_break(2000);
      }
      current_angle_gyro += turn_direction * 90;
      turns++;
      delay_walls = 2500; // 2500
      last_rotate = millis();
    }
  }
  CASE = PID;
}

void loop_function() {
  // measure the processing frequency
  // uint32_t t = millis() - time_elapsed;
  // if (t > 100) {
  //   Serial.println(((double)freq / t) * 1000);
  //   time_elapsed = millis();
  //   freq = 0;
  // }

  read_gyro(false);
  switch(CASE) {
    case PID: {
      double err = current_angle_gyro - gx;
      if (millis() - last_rotate > 1500 && turns >= 12) {
        CASE = STOP;
      }
      else {
        pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;
        move_servo(pid_error_gyro);
      }
      move_motor(MOTOR_SPEED);
      break;
    }
    case FOLLOW_CUBE: {
      if (millis() - last_rotate > 1500 && turns >= 12) {
        CASE = STOP;
      }
      move_servo(avoid_cube_angle);
      move_motor(MOTOR_SPEED);
      break;
    }
    case AFTER_CUBE: {
      if (millis() - last_rotate > 1500 && turns >= 12) {
        CASE = STOP;
      }
      double err = current_angle_gyro - gx + cube_last * CORRECTION_ANGLE;
      if (abs(err) < 5) {
        move_cm_gyro(12, MOTOR_SPEED, current_angle_gyro + cube_last * CORRECTION_ANGLE);
        // motor_break(2000);
        CASE = PID;
      }
      else {
        pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;
        move_servo(pid_error_gyro);
      }
      move_motor(MOTOR_SPEED);
      break;
    }
    case STOP: {
      move_until_angle(MOTOR_SPEED, current_angle_gyro);
      move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro);
      is_running = false;
      Serial.println("Stop case");
      motor_break(100000);
      break;
    }
    default: {
      break;
    }
  }

  if(millis() - last_gyro_read > 10) {
    // Serial.print("angle: ");
    // Serial.print(current_angle_gyro);
    // Serial.print("\tgx: ");
    // Serial.println(gx);
    last_gyro_read = millis();
  }

  update_servo();

  // prioritize executing pending commands
  while (Serial0.available() > 0) { // if we have some characters waiting
    char receivedChar = Serial0.read(); // we get the first character
    if (receivedChar == '\n') { // if it's the end of message marker
      execute(receivedMessage); // execute the received command from the OpenMV camera
      receivedMessage = ""; // Reset the received message
    }
    else {
      receivedMessage += receivedChar; // Append characters to the received message
    }
  }

  // freq++;
}