// -----Pid logic-----
#define PID 0
#define STOP 1
#define FOLLOW_CUBE 2
#define PASS_CUBE 3
#define CORRECTION_ANGLE 30
#define FORWARD_DISTANCE1 40
#define FORWARD_DISTANCE2 55

double avoid_cube_angle = 0;

int CASE = PID;
int OUTER_WALL_DIRECTION = 0;
int GYRO_OFFSET = 5;
int turn_direction = 0;
int motor_speed = 60;
int pass_speed = 60;
int start_avoid_dis1 = 0;
int start_avoid_dis2 = 0;

int loop_cnt = 0;
int current_angle_gyro = 0;

double kp_gyro = 0.025;
double ki_gyro = 0;
double kd_gyro = 0.042;
double pid_error_gyro, pid_last_error_gyro = 0;

bool flag = false;
int last_cube_color = 0;

void execute(String cmd) {
  if (cmd[0] == 'R' || cmd[0] == 'G') {
    start_avoid_dis1 = read_motor_cm();
    CASE = PASS_CUBE;
    
    if (cmd[0] == 'R') {
      last_cube_color = 1;
    }
    else {
      last_cube_color = -1;
    }
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
  }
  else {
    int msg = (int)val;
    if (msg) {
      CASE = PID;
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
    case FOLLOW_CUBE: {
      move_servo(avoid_cube_angle);
      move_motor(motor_speed);
      break;
    }
    case PASS_CUBE: {
      if (read_motor_cm() - start_avoid_dis1 <= FORWARD_DISTANCE1) {
        move_servo(last_cube_color * -0.5);
        move_motor(pass_speed);
        start_avoid_dis2 = read_motor_cm();
      }
      else if (read_motor_cm() - start_avoid_dis2 <= FORWARD_DISTANCE2) {
        move_servo(last_cube_color * 1);
        move_motor(pass_speed);
      }
      else if (abs(current_angle_gyro - gx) < 1)  {
        CASE = PID;
      }
      else {
        pid_error_gyro = (current_angle_gyro  - gx) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;

        move_servo(pid_error_gyro);
        move_motor(pass_speed);
      }
      break;
    }
    case STOP: {
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
      if (CASE != PASS_CUBE) {
        execute(receivedMessage); // execute the received command from the OpenMV camera
      }
      receivedMessage = ""; // Reset the received message
    }
    else {
      receivedMessage += receivedChar; // Append characters to the received message
    }
  }

  // freq++;
}