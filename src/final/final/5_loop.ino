// -----Units of measurement-----
// The velocities are on a -100 to 100 scale
// The distances are in cm
// The time is in ms
// The angles are in degrees

#define QUALI false

// -----Cases-----
#define PID 0
#define STOP 1
#define FOLLOW_CUBE 2
#define AFTER_CUBE 3
#define FIND_PARKING 4
int CASE = PID;
bool TURNED = false;

// -----Angles-----
#define GYRO_OFFSET_PARKING 0
#define CORRECTION_ANGLE 45
#define AVOIDANCE_ANGLE 45
double avoid_cube_angle = 0;

// -----Distances-----
#define CORNER_DISTANCE_FINAL 0
#define CORNER_DISTANCE_QUALI 42 // maybe 40 if the battery is overcharged
#define CORNER_DISTANCE_PARKING 70

// -----Velocities-----
#define MOTOR_SPEED 100

// -----Logic-----
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
  if (!valid_command(cmd))
    return;

  int pos = 0, sign = 1;
  if (cmd[pos] == 'r' || cmd[pos] == 'g')
    pos++;
  // manually going over the signs since the .toDouble function wouldn't parse them on its own
  if (cmd[pos] == '+')
    sign = 1, pos++;
  else if (cmd[pos] == '-')
    sign = -1, pos++;
  double val = cmd.substring(pos).toDouble();

  if (!QUALI) {
    if (CASE == FIND_PARKING && cmd[0] == 'P') {
      // parking sequence
      return;
    }

    if (CASE != FIND_PARKING) {
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
      if (cmd[0] == 'r' || cmd[0] == 'g') {
        avoid_cube_angle = val * sign;
        CASE = FOLLOW_CUBE;
        return;
      }
    }
  }
  
  int msg = (int)val;
  if (msg) {
    if (turn_direction == 0) {
      if (msg == 1) { // blue line
        turn_direction = 1;
      }
      else { // orange line
        turn_direction = -1;
      }
    }
    if (millis() - last_rotate > delay_walls) {
      if (CASE == FIND_PARKING) {
        move_cm_gyro(CORNER_DISTANCE_PARKING, MOTOR_SPEED, current_angle_gyro);
        // motor_break(1000000);
      }
      else if (abs(current_angle_gyro - gx) < 10) {
        if (QUALI)
          move_cm_gyro(CORNER_DISTANCE_QUALI, MOTOR_SPEED, current_angle_gyro);
        else
          move_cm_gyro(CORNER_DISTANCE_FINAL, MOTOR_SPEED, current_angle_gyro);
      }
      else if (CASE != FOLLOW_CUBE) {
        move_until_angle(MOTOR_SPEED, current_angle_gyro + turn_direction * 20);
        move_cm_gyro(7, MOTOR_SPEED, current_angle_gyro);
      }
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
      if (!TURNED && turns == 8 && cube_last == 1) {
        if ((current_angle_gyro - turn_direction * 90) - gx > 0) {
          current_angle_gyro += turn_direction * 90;
        }
        else {
          current_angle_gyro -= turn_direction * 270;
        }
        turn_direction *= -1;
        TURNED = true;
      }
      double err = current_angle_gyro - gx;
      if (millis() - last_rotate > 1500 && turns >= 11) {
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
      if (!TURNED && turns == 8 && cube_last == 1) {
        if ((current_angle_gyro - turn_direction * 90) - gx > 0) {
          current_angle_gyro += turn_direction * 90;
        }
        else {
          current_angle_gyro -= turn_direction * 270;
        }
        turn_direction *= -1;
        TURNED = true;
        CASE = PID;
      }
      if (millis() - last_rotate > 1500 && turns >= 11) {
        CASE = STOP;
      }
      else {
        move_servo(avoid_cube_angle);
        move_motor(MOTOR_SPEED);
      }
      break;
    }
    case AFTER_CUBE: {
      if (!TURNED && turns == 8 && cube_last == 1) {
        if ((current_angle_gyro - turn_direction * 90) - gx > 0) {
          current_angle_gyro += turn_direction * 90;
        }
        else {
          current_angle_gyro -= turn_direction * 270;
        }
        turn_direction *= -1;
        TURNED = true;
        CASE = PID;
      }
      if (millis() - last_rotate > 1500 && turns >= 11) {
        CASE = STOP;
      }
      else {
        double err = current_angle_gyro - gx + cube_last * CORRECTION_ANGLE;
        if (abs(err) < 5) {
          move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro + cube_last * CORRECTION_ANGLE);
          CASE = PID;
        }
        else {
          pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;
          move_servo(pid_error_gyro);
        }
      }
      move_motor(MOTOR_SPEED);
      break;
    }
    case STOP: {
      move_until_angle(MOTOR_SPEED, current_angle_gyro);
      motor_break(3000);
      CASE = FIND_PARKING;
      break;
    }
    case FIND_PARKING: {
      double err = current_angle_gyro - turn_direction * GYRO_OFFSET_PARKING - gx;
      pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
      pid_last_error_gyro = pid_error_gyro;
      move_servo(pid_error_gyro);
      move_motor(MOTOR_SPEED);
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

  // execute pending commands
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