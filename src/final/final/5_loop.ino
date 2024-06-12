// -----Pid logic-----
#define PID 0
#define STOP 1
#define AVOID_CUBE 2
#define CORRECTION_ANGLE 35

double avoid_cube_angle = 0;

int CASE = PID;
int OUTER_WALL_DIRECTION = 0;
int GYRO_OFFSET = 5;
int turn_direction = 0;
int motor_speed = 80;
int pass_speed = 40;

int loop_cnt = 0;
int current_angle_gyro = 0;

double kp_gyro = 0.025;
double ki_gyro = 0;
double kd_gyro = 0.042;
double pid_error_gyro, pid_last_error_gyro = 0;

bool flag = false;
char last_cube_color = 0;

void pass_cube() {
  servo.write(ANGLE_MID);
  int start_dis = read_motor_cm(WHEEL_DIAM, GEAR_RATIO);
  // Serial.println(start_dis);
  while (read_motor_cm(WHEEL_DIAM, GEAR_RATIO) - start_dis < 45)
    move_motor(pass_speed);
  
  read_gyro(false);
  pid_error_gyro = ((current_angle_gyro + last_cube_color * CORRECTION_ANGLE) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
  pid_last_error_gyro = pid_error_gyro;
  
  while (abs(pid_error_gyro) >= 0.1) {
    move_servo(pid_error_gyro);
    update_servo();
    move_motor(pass_speed);

    read_gyro(false);
    pid_error_gyro = ((current_angle_gyro + last_cube_color * CORRECTION_ANGLE) - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
    pid_last_error_gyro = pid_error_gyro;
  }
}

void execute(String cmd) {
  if (cmd[0] == 'a') {
    pass_cube();
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
    CASE = AVOID_CUBE;
    last_cube_color = -1;
    if (cmd[0] == 'r') {
      last_cube_color = 1;
    }
  }
  else {
    int msg = (int)val;
    if (msg) {
      last_cube_color = 0;
      flag = 1;
      if (msg == 1) { // blue line
        turn_direction = 1;
        GYRO_OFFSET = 5;
      }
      else { // orange line
        turn_direction = -1;
        GYRO_OFFSET = 3;
      }
      OUTER_WALL_DIRECTION = -turn_direction;
    }
    else {
      flag = turn_direction = 0;
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
      move_motor(motor_speed);
      double err = current_angle_gyro - gx + GYRO_OFFSET * OUTER_WALL_DIRECTION;
      if (abs(err) < 10 && millis() - last_rotate > 1500 && turns >= 12) {
        CASE = STOP;
      }
      else if (abs(err) < 10 && millis() - last_rotate > 1500 && flag != 0) {
        current_angle_gyro += turn_direction * 90;
        turns++;
        last_rotate = millis();
      }
      else {
        pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;
        move_servo(pid_error_gyro);
      }
      break;
    }
    case AVOID_CUBE: {
      move_servo(avoid_cube_angle);
      move_motor(motor_speed);
      break;
    }
    case STOP: {
      // delay(500);
      motor_break();
      Serial.println("Stop case");
      delay(100000);
      is_running = false;
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