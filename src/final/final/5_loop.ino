// -----Cases-----
#define PID 0
#define STOP 1
#define FOLLOW_CUBE 2
#define AFTER_CUBE 3

// -----Velocities and distances-----
#define CORRECTION_ANGLE 50
#define AVOIDANCE_ANGLE 50
#define FORWARD_DISTANCE 30
#define CORNER_DISTANCE_FINAL 28
#define CORNER_DISTANCE_QUALI 42 // maybe 40 if the battery is overcharged
#define MOTOR_SPEED 100

double avoid_cube_angle = 0;

int CASE = PID;
int turn_direction = 0;
int delay_walls = 250;
int cube_last = 0;

void pass_cube(int cube_last) {
  // motor_break(1000);
  move_until_angle(MOTOR_SPEED, cube_last * -AVOIDANCE_ANGLE);
  // motor_break(1000);
  move_cm_gyro(12.5, MOTOR_SPEED, cube_last * -AVOIDANCE_ANGLE);
  // motor_break(1000);
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
    // if (cmd[0] == 'g')
    //   motor_break(10000);
    return;
  } else {
    CASE = PID;
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
        move_cm_gyro(CORNER_DISTANCE_FINAL, MOTOR_SPEED, 0); // change according to the task
      }
      else {
        move_until_angle(MOTOR_SPEED, turn_direction * 15);
      }
      // else if ((current_angle_gyro - gx) * turn_direction >= 10 * turn_direction) {
      //   move_cm_gyro(20, MOTOR_SPEED, 0);
      //   motor_break(100);
      //   move_until_angle(-MOTOR_SPEED, -turn_direction * 90);
      //   motor_break(100);
      //   move_cm_gyro(25, MOTOR_SPEED, -turn_direction * 90);
      //   move_until_angle(MOTOR_SPEED, turn_direction * 90);
      //   // motor_break(1000);
      // }
      // else {
      //   move_until_angle(MOTOR_SPEED, 0);
      //   // motor_break(100);
      //   // move_cm_gyro(3, -MOTOR_SPEED, 0);
      //   // motor_break(100);
      //   // motor_break(3000);
      // }
      current_angle_gyro += turn_direction * 90;
      turns++;
      delay_walls = 2500;
      last_rotate = millis();
    }
  }
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
      // if (current_angle_gyro == turn_direction * 90 && turn_direction != 0 && abs(err) < 10)
      //   motor_break(1000000000);
      if (abs(err) < 10 && millis() - last_rotate > 1500 && turns >= 12) {
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
      move_servo(avoid_cube_angle);
      move_motor(MOTOR_SPEED);
      break;
    }
    case AFTER_CUBE: {
      double err = current_angle_gyro - gx + cube_last * CORRECTION_ANGLE;
      if (abs(err) < 10) {
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
      delay(500);
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