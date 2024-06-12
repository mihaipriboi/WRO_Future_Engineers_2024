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