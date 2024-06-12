// -----Pid logic-----
#define PID 0
#define STOP 1
#define GYRO_OFFSET 1

double avoid_cube_angle = 0;

int CASE = PID;
int OUTER_WALL_DIRECTION = 0;
int turn_direction = 0;
int motor_speed = 80; // 40

int loop_cnt = 0;
int current_angle_gyro = 0;

double kp_gyro = 0.025;
double ki_gyro = 0;
double kd_gyro = 0.042;
double pid_error_gyro, pid_last_error_gyro = 0;

bool flag = false;

void execute(String cmd) {
  int msg = cmd[0] - '0';
  if (msg) {
    CASE = PID;
    flag = 1;
    if (msg == 1) { // blue line
      turn_direction = 1;
    }
    else { // orange line
      turn_direction = -1;
    }
    OUTER_WALL_DIRECTION = -turn_direction;
  }
  else {
    flag = turn_direction = 0;
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
    } else {
      receivedMessage += receivedChar; // Append characters to the received message
    }
  }

  // freq++;
}