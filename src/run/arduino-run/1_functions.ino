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

void flush_messages() {
  while (Serial0.available() > 0) { // if we have some characters waiting
    Serial0.read(); // flush the character
  }
}

void custom_delay(long long delay_time) { // delay function that flushes all of the data
  long long start_time = millis();
  while (millis() - start_time < delay_time) {
    read_gyro(false);
    flush_messages();
  }
}

bool valid_command(String cmd) { // function that checks the validity of a command received from the camera
  if ('0' <= cmd[0] && cmd[0] <= '9') {
    if (cmd[0] > '2' || cmd[0] == '0')
      return false;
    if (cmd.length() != 1)
      return false;
  }
  if (cmd[0] != 'r' || cmd[0] != 'g' || cmd[0] != 'R' || cmd[0] != 'G')
    return false;
  return true;
}