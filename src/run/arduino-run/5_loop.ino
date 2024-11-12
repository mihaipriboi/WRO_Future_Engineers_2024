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
#define STOP_BEFORE_PARKING 5
#define PARK 6

int CASE = PID;
bool TURNED = false;

// -----Angles-----
#define CORRECTION_ANGLE 45
#define AVOIDANCE_ANGLE 45
double follow_cube_angle = 0;

// -----Distances-----
#define CORNER_DISTANCE_FINAL 0
#define CORNER_DISTANCE_QUALI 42 // maybe 40 if the battery is overcharged
#define CORNER_DISTANCE_PARKING 60
#define CORNER_DISTANCE_PARKING_FIRST_TURN 70

// -----Velocities-----
#define MOTOR_SPEED 100

// -----Times-----
#define TURNAROUND_DELAY 1600
#define FIRST_STOP_DELAY 1500

// -----Logic-----
int turn_direction = 0;
int delay_walls = 250;
int cube_last = 0;
int turns_parking = 0;
int turns = 0;

// -----Debug-----
long last_gyro_read = 0;
int freq = 0;

// hardcoded sequence that avoids a cube
void pass_cube(int cube_last) {
  read_gyro(false);
  int start_angle = gx;
  move_until_angle(MOTOR_SPEED, start_angle + cube_last * -AVOIDANCE_ANGLE);
  move_cm_gyro(7, MOTOR_SPEED, start_angle + cube_last * -AVOIDANCE_ANGLE);
  CASE = AFTER_CUBE;
}

// function that parses a command and executes it
void execute(String cmd) {
  if (!valid_command(cmd))
    return;

  // the following sequence gets the number from the command (if available)
  int pos = 0, sign = 1;
  if (cmd[pos] == 'r' || cmd[pos] == 'g')
    pos++;
  // manually going over the signs since the .toDouble function wouldn't parse them on its own
  if (cmd[pos] == '+')
    sign = 1, pos++;
  else if (cmd[pos] == '-')
    sign = -1, pos++;
  double val = cmd.substring(pos).toDouble();

  if (!QUALI) { // if we're in the final round
    if (CASE == FIND_PARKING && cmd[0] == 'P') { // if we're searching for the parking spot and we find it
      CASE = PARK; // park the robot
      return;
    }

    if (cmd[0] == 'P') // if we see the parking slot, but we didn't finish the obstacle round we ignore it
      return;

    if (CASE != FIND_PARKING) {
      if (cmd[0] == 'R' || cmd[0] == 'G') { // if we're in the proximity of a cube
        if (cmd[0] == 'R') { // we determine the direction in which we avoid the cube
          cube_last = 1;
        }
        else {
          cube_last = -1;
        }
        pass_cube(cube_last);
        return;
      }
      if (cmd[0] == 'r' || cmd[0] == 'g') { // if we see a cube but we're not close enough to avoid it
        follow_cube_angle = val * sign;
        CASE = FOLLOW_CUBE;
        return;
      }
    }
  }
  
  int msg = (int)val;
  if (msg) { // a turn was detected
    if (turn_direction == 0) { // if we don't know the direction yet
      if (msg == 1) { // blue line
        turn_direction = 1;
      }
      else { // orange line
        turn_direction = -1;
      }
    }
    if (millis() - last_rotate > delay_walls) { // if we can make a turn
      if (CASE == FIND_PARKING) { // if we're searching for the parking spot
        // if we're at the first turn, we have to move more in order to position ourselves close to the outer walls
        if (turns_parking == 0)
          move_cm_gyro(CORNER_DISTANCE_PARKING_FIRST_TURN, MOTOR_SPEED, current_angle_gyro);
        else
          move_cm_gyro(CORNER_DISTANCE_PARKING, MOTOR_SPEED, current_angle_gyro);
        turns_parking++;
      }
      else if (abs(current_angle_gyro - gx) < 10) { // if we're during the obstacle round or in the quali and we're straight
        // position ourselves in order to not hit the walls
        if (QUALI)
          move_cm_gyro(CORNER_DISTANCE_QUALI, MOTOR_SPEED, current_angle_gyro);
        else
          move_cm_gyro(CORNER_DISTANCE_FINAL, MOTOR_SPEED, current_angle_gyro);
      }
      else if (CASE != FOLLOW_CUBE) { // if we're crooked after avoiding a cube we position ourselves for the turn
        move_until_angle(MOTOR_SPEED, current_angle_gyro + turn_direction * 20);
        move_cm_gyro(7, MOTOR_SPEED, current_angle_gyro);
      }
      current_angle_gyro += turn_direction * 90; // update the goal angle for the next sequence
      turns++; // increase the number of turns made
      delay_walls = 2500; // larger delay for every turn except the first one
      // as we may have the starting position close to the first turn
      last_rotate = millis(); // update the last time we turned
    }
  }
}

void check_and_execute_turnaround(double gx) {
  // if we didn't do the turnaround yet
  // and we did 2 runs of the map
  // and the last seen cube is red
  // and some time passed since the 8th turn
  // (so that we can see the first cube in the starting sequence in case this sequence had 2 cubes and we spawned between them)
  if (QUALI)
    return;
  if (!TURNED && turns == 8 && cube_last == 1 && millis() - last_rotate > TURNAROUND_DELAY) {
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
      check_and_execute_turnaround(gx);
      double err = current_angle_gyro - gx;
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the round
        if (!QUALI) {
          CASE = STOP_BEFORE_PARKING; // we need to stop and search for the parking
        }
        else {
          CASE = STOP; // stop, challenge over
        }
      }
      else {
        // classic pid on the gyro so that we can move straight
        pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
        pid_last_error_gyro = pid_error_gyro;
        move_servo(pid_error_gyro);
      }
      move_motor(MOTOR_SPEED);
      break;
    }

    case FOLLOW_CUBE: {
      check_and_execute_turnaround(gx);
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the obstacle round, we need to stop and search for the parking
        CASE = STOP_BEFORE_PARKING;
      }
      else {
        // write to the servo the pid computed on the camera in order to follow the cube
        move_servo(follow_cube_angle);
        move_motor(MOTOR_SPEED);
      }
      break;
    }

    case AFTER_CUBE: {
      check_and_execute_turnaround(gx);
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the obstacle round, we need to stop and search for the parking
        CASE = STOP_BEFORE_PARKING;
      }
      else {
        double err = current_angle_gyro - gx + cube_last * CORRECTION_ANGLE;
        if (abs(err) < 5) {
          // after we avoid the cube, move forward a bit more so that we're positioned
          // to see the next cube
          move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro + cube_last * CORRECTION_ANGLE);
          CASE = PID;
        }
        else {
          // classic pid on the gyro so that we can move in the opposite direction
          // so that we can see the next cube
          pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
          pid_last_error_gyro = pid_error_gyro;
          move_servo(pid_error_gyro);
        }
      }
      move_motor(MOTOR_SPEED);
      break;
    }

    case STOP: {
      // we finished the challenge, stop the robot
      move_until_angle(MOTOR_SPEED, current_angle_gyro);
      move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro);
      is_running = false;
      Serial.println("Stop case");
      motor_break(100000);
    }

    case STOP_BEFORE_PARKING: {
      // straighten ourselves, start searching for the parking
      move_until_angle(MOTOR_SPEED, current_angle_gyro);
      turns_parking = 0;
      CASE = FIND_PARKING;
      break;
    }

    case PARK: {
      // hardcoded sequence of moves that positions us in the parking spot
      move_until_angle(MOTOR_SPEED, current_angle_gyro + turn_direction * 55);
      move_until_angle(MOTOR_SPEED, current_angle_gyro - turn_direction * 90);
      CASE = STOP;
      break;
    }

    case FIND_PARKING: {
      // classic pid on the gyro so that we can move straight
      double err = current_angle_gyro - gx;
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

  // if(millis() - last_gyro_read > 10) {
    // Serial.print("angle: ");
    // Serial.print(current_angle_gyro);
    // Serial.print("\tgx: ");
    // Serial.println(gx);
    // last_gyro_read = millis();
  // }

  update_servo();

  // execute pending commands
  while (Serial0.available() > 0) { // if we have some characters waiting
    char receivedChar = Serial0.read(); // we get the first character
    if (receivedChar == '\n') { // if it's the end of message marker
      if (CASE != PARK) { // if we want to execute commands
        execute(receivedMessage); // execute the received command from the OpenMV camera
      }
      receivedMessage = ""; // reset the received message
    }
    else {
      receivedMessage += receivedChar; // append characters to the received message
    }
  }

  // freq++;
}