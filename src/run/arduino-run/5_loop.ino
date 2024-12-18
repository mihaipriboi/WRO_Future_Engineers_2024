// -----Units of measurement-----
// The velocities are on a -100 to 100 scale
// The distances are in cm
// The time is in ms
// The angles are in degrees

// -----Cases-----
#define PID 0
#define FOLLOW_CUBE 1
#define AFTER_CUBE 2
#define STOP_BEFORE_FIND_PARKING 3
#define POSITION_BEFORE_FIND_PARKING 4
#define FIND_PARKING 5
#define POSITION_FOR_PARK 6
#define PARK 7
#define STOP_FINAL 8
#define STOP_QUALI 9

int CASE = PID;

// -----Angles-----
#define CORRECTION_ANGLE 30
#define AVOIDANCE_ANGLE 45
#define TURNAROUND_ANGLE 50
double follow_cube_angle = 0;

// -----Distances-----
#define CORNER_DISTANCE_FINAL 0
#define CORNER_DISTANCE_QUALI 42 // maybe 40 if the battery is overcharged
#define CORNER_DISTANCE_PARKING 45

// -----Velocities-----
#define MOTOR_SPEED 100
#define PARKING_SPEED 70

// -----Times-----
#define TURNAROUND_DELAY 1500
#define FIRST_STOP_DELAY 1500
#define FOLLOW_CUBE_DEAD_TIME 250
#define PASS_CUBE_DELAY 200

// -----Logic-----
int turn_direction = 0;
int delay_walls = 250;
int cube_last = 0;
int turns = 0;
long last_follow_cube = 0;
long last_cube_time = 0;
int is_front = 0;
int no_cubes = 0;
int before_cube = 0;
int first_cube = 0;

int before_side_last_cube = 0; // the color of the last cube from the side that comes before the starting side (0 if non-existent)
bool turned = false; // if we did the turnaround or not

// -----Debug-----
long last_gyro_read = 0;
int freq = 0;

// hardcoded sequence that avoids a cube
void pass_cube(int cube_last) {
  read_gyro(false);
  double angle_addition = 0;
  double start_angle = gx;
  double err = abs(current_angle_gyro - start_angle);

  // if (err >= 50) { // if we're really crooked, substract a given value so that we don't overcompensate
  //   angle_addition -= 10;
  // } 
  // else {
  //   angle_addition -= max(err / 2.75, 0.0); // if we're crooked, dinamically adjust the avoidance angle so that we don't overshoot
  // }
  if (cube_last == -1)
    angle_addition += 8;
  move_until_angle(MOTOR_SPEED, start_angle - cube_last * (AVOIDANCE_ANGLE + angle_addition)); // steer away from the cube
  start_angle = gx;

  // move_cm_gyro(12, MOTOR_SPEED, current_angle_gyro + cube_last * 30);

  last_cube_time = millis();
  CASE = AFTER_CUBE;
}

// hardcoded sequence that turns around
// we need the last passed cube so that we know which way we take the turn so that we don't hit anything
void turn_around(int cube_last) {
  read_gyro(false);
  double start_angle = gx;
  move_until_angle(MOTOR_SPEED, start_angle - cube_last * 57);
  move_until_angle(MOTOR_SPEED, current_angle_gyro + cube_last * 200);
  current_angle_gyro += cube_last * 180;
  turn_direction *= -1;
  turned = true;
  CASE = PID;
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

  if (FINAL) { // if we're in the final round
    if (CASE == FIND_PARKING && cmd[0] == 'P') { // if we're searching for the parking spot and we find it
      CASE = POSITION_FOR_PARK; // park the robot
      return;
    }

    if (cmd[0] == 'P') // if we see the parking slot, but we didn't finish the obstacle round we ignore it
      return;

    // if(cmd[0] == 'S') {
    //   if(cmd[1] == 'L')
    //     move_until_angle(MOTOR_SPEED, current_angle_gyro - 20); // straighten ourselves
    //   else
    //     move_until_angle(MOTOR_SPEED, current_angle_gyro + 20); // straighten ourselves
    // }

    if (CASE == POSITION_BEFORE_FIND_PARKING && cmd[0] == 'W') { // if we're positioning ourselves close to the wall and we're in its proximity
      move_until_angle(MOTOR_SPEED, current_angle_gyro); // straighten ourselves
      CASE = FIND_PARKING; // start searching for the parking lot
      return;
    }

    if (CASE == PARK && (cmd[0] == 'W' && cmd[1] == 'P')) { // if we're positioning ourselves close to the wall and we're in its proximity
      move_cm_gyro(60, PARKING_SPEED, current_angle_gyro - turn_direction * 90); // position ourselves closer
      CASE = STOP_FINAL; // we finished the parking, stop
      return;
    }

    if (cmd[0] == 'W') // if we see the outer wall, but we don't need it we ignore it
      return;

    if (CASE != FIND_PARKING && CASE != POSITION_BEFORE_FIND_PARKING) {
      if ((cmd[0] == 'R' || cmd[0] == 'G') && millis() - last_cube_time >= PASS_CUBE_DELAY) { // if we're in the proximity of a cube
        if (turns == 7)
          before_side_last_cube = cube_last; // remember the last passed cube (excluding this one)
        if (cmd[0] == 'R') { // we determine the direction in which we avoid the cube and remember its color
          cube_last = 1;
        }
        else {
          cube_last = -1;
        }

        if(turns == 0)
          is_front = 1;

        if(turns == 4) {
          no_cubes++;
          if(first_cube == 0)
            first_cube = cube_last;
        }

        if(turns == 7) {
          before_cube = cube_last;
        }

        if(turns == 8) {
          if(is_front == 0)
            before_cube = cube_last;
          else if(no_cubes == 2) {
            before_cube = first_cube;
          }
        } 

        if (turns == 8 && before_cube == 1 && !turned) // if we have are in the position to turn around, the cube is red and we haven't turned around already
          turn_around(cube_last);
        else 
          pass_cube(cube_last);
        return;
      }
      if (cmd[0] == 'r' || cmd[0] == 'g') { // if we see a cube but we're not close enough to avoid it
        follow_cube_angle = val * sign;
        CASE = FOLLOW_CUBE;
        last_follow_cube = millis();
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
        move_cm_gyro(CORNER_DISTANCE_PARKING, MOTOR_SPEED, current_angle_gyro);
      }
      else if (abs(current_angle_gyro - gx) < 10) { // if we're during the obstacle round or in the quali and we're straight
        // position ourselves in order to not hit the walls
        if (FINAL)
          move_cm_gyro(CORNER_DISTANCE_FINAL, MOTOR_SPEED, current_angle_gyro);
        else
          move_cm_gyro(CORNER_DISTANCE_QUALI, MOTOR_SPEED, current_angle_gyro);
      }
      else if (CASE == AFTER_CUBE) { // if we're crooked after avoiding a cube we position ourselves for the turn
        if (-cube_last == turn_direction) { // if we passed by it in the turn's direction then we just need to straighted ourselves for more manuver room
          move_until_angle(MOTOR_SPEED, current_angle_gyro);
        }
        else {
          move_until_angle(MOTOR_SPEED, current_angle_gyro + turn_direction * 30);
          move_cm_gyro(7, MOTOR_SPEED, current_angle_gyro + turn_direction * 30);
        }
        CASE = PID;
      }
      current_angle_gyro += turn_direction * 90; // update the goal angle for the next sequence
      turns++; // increase the number of turns made
      delay_walls = 2300; // larger delay for every turn except the first one
      // as we may have the starting position close to the first turn
      last_rotate = millis(); // update the last time we turned
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
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the round
        if (FINAL) {
          CASE = STOP_BEFORE_FIND_PARKING; // we need to stop and search for the parking
        }
        else {
          CASE = STOP_QUALI; // stop, challenge over
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
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the obstacle round, we need to stop and search for the parking
        CASE = STOP_BEFORE_FIND_PARKING;
      }
      else {
        if (millis() - last_follow_cube > FOLLOW_CUBE_DEAD_TIME) // if we lost the cube, we just go back to the default PID case
          CASE = PID;
        // write to the servo the pid computed on the camera in order to follow the cube
        move_servo(follow_cube_angle);
        move_motor(MOTOR_SPEED);
      }
      break;
    }

    case AFTER_CUBE: {
      if (millis() - last_rotate > FIRST_STOP_DELAY && turns >= 12) { // if we did 3 runs of the obstacle round, we need to stop and search for the parking
        CASE = STOP_BEFORE_FIND_PARKING;
      }
      else {
        int angle_addition = 0;
        if (cube_last == 1) // due to a slight asymmetry in the steering, when avoiding red cubes we need to steer less
          angle_addition = -9;
        double err = current_angle_gyro - gx + cube_last * (CORRECTION_ANGLE + angle_addition);
        if (abs(err) < 5) {
          // after we avoid the cube, move forward a bit more so that we're positioned
          // to see the next cube
          if (cube_last == turn_direction) // compensate less on the inside
            move_cm_gyro(5, MOTOR_SPEED, current_angle_gyro + cube_last * (CORRECTION_ANGLE + angle_addition));
          else
            move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro + cube_last * (CORRECTION_ANGLE + angle_addition));
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

    case STOP_BEFORE_FIND_PARKING: {
      // straighten ourselves, start positioning for the parking
      if (no_cubes == 1)
        move_cm_gyro(22, MOTOR_SPEED, current_angle_gyro);
      else
        move_cm_gyro(17, MOTOR_SPEED, current_angle_gyro);
      motor_break(5000);
      CASE = POSITION_BEFORE_FIND_PARKING;
      break;
    }

    case POSITION_BEFORE_FIND_PARKING: {
      // classic pid on the gyro so that we can move perpendicular to the walls
      // we don't just call the move_until_angle function so that we can still execute commands
      double err = current_angle_gyro - gx - turn_direction * 90;
      pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
      pid_last_error_gyro = pid_error_gyro;
      move_servo(pid_error_gyro);
      move_motor(MOTOR_SPEED);
      break;
    }

    case FIND_PARKING: {
      // classic pid on the gyro so that we can move straight
      // basically immitating a quali run until we find the parking spot
      double err = current_angle_gyro - gx;
      pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
      pid_last_error_gyro = pid_error_gyro;
      move_servo(pid_error_gyro);
      move_motor(MOTOR_SPEED);
      break;
    }

    case POSITION_FOR_PARK: {
      // hardcoded sequence of moves that positions us in the parking spot
      // after that, we just get closer to the outside wall so that we're fully in
      
      move_cm_gyro(3, PARKING_SPEED, current_angle_gyro);
      drift(PARKING_SPEED, turn_direction, current_angle_gyro + turn_direction * 90);
      move_cm_gyro(16, -PARKING_SPEED, current_angle_gyro + turn_direction * 90);
      drift(PARKING_SPEED, -turn_direction, current_angle_gyro - turn_direction * 90);
      CASE = PARK;
      break;
    }

    case PARK: {
      // classic pid on the gyro so that we can move straight into the parking space
      // we don't just call the move_until_angle function so that we can still execute commands
      double err = current_angle_gyro - gx - turn_direction * 90;
      pid_error_gyro = (err) * kp_gyro + (pid_error_gyro - pid_last_error_gyro) * kd_gyro;
      pid_last_error_gyro = pid_error_gyro;
      move_servo(pid_error_gyro);
      move_motor(PARKING_SPEED);
      break;
    }
    
    case STOP_FINAL: {
      // we finished the challenge, stop the robot
      is_running = false;
      Serial.println("Stop case");
      motor_break(1000000000);
    }

    case STOP_QUALI: {
      // we finished the challenge, stop the robot
      move_until_angle(MOTOR_SPEED, current_angle_gyro);
      move_cm_gyro(10, MOTOR_SPEED, current_angle_gyro);
      is_running = false;
      Serial.println("Stop case");
      motor_break(1000000000);
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
      if (CASE != POSITION_FOR_PARK && CASE != STOP_FINAL && CASE != STOP_QUALI && CASE != STOP_BEFORE_FIND_PARKING) { // if we want to execute commands
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