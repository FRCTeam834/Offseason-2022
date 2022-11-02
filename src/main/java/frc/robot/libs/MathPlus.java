package frc.robot.libs;

public class MathPlus {
  /**
   * Is value in the interval [-range, range]
   */
  public static final boolean inInclusiveRange(double value, double range) {
    return (value <= range && value >= -range);
  }

  /**
   * Returns normalized difference between two angles [-180, 180)
   * !Note: This is b - a
   * @param a deg
   * @param b deg
   * @return deg
   */
  public static final double realAngleDiff(double b, double a) {
    double d = (b - a) % 360.0;
    if (d < -180) d += 360;
    else if (d >= 180) d -= 360;
    return d;
  }

  /**
   * Returns magnitude of normalized difference between two angles [-180, 180)
   * @param a deg
   * @param b deg
   * @return deg
   */
  public static final double absRealAngleDiff(double b, double a) {
    return Math.abs(realAngleDiff(b, a));
  }

  /**
   * 
   * Matches targetAngle to (0, 360] scope of scopeAngle
   * @param targetAngle
   * @param scopeAngle
   * @return
   */
  public static final double matchAngleScope(double targetAngle, double scopeAngle) {
    if (targetAngle < scopeAngle) {
      while (Math.abs(targetAngle - scopeAngle) >= 180) {
        targetAngle += 360;
      }
    } else {
      while (Math.abs(targetAngle - scopeAngle) >= 180) {
        targetAngle -= 360;
      }
    }
    return targetAngle;
  }

  /** Optimize angle so swerve module never has to turn more than 90 degrees */
  public static final double optimizeSwerveAngle(double targetAngle, double currentAngle) {
    double a = matchAngleScope(targetAngle, currentAngle); // Case 1
    double b = matchAngleScope(targetAngle - 180, currentAngle); // Case 2

    //if (absRealAngleDiff(a, currentAngle) > absRealAngleDiff(b, currentAngle)) {
    if (Math.abs(a - currentAngle) > Math.abs(b - currentAngle)) {
      return b;
    }
    return a;
  }
  /**
   * 
   * Optimize angle so swerve module never has to turn more than <threshold> degrees
   * @param targetAngle
   * @param scopeAngle
   * @return
   */
  public static final double optimizeSwerveAngle(double targetAngle, double currentAngle, double threshold) {
    double a = matchAngleScope(targetAngle, currentAngle); // Case 1
    double b = matchAngleScope(targetAngle - 180, currentAngle); // Case 2

    // Difference has not reached threshold, do not consider option 2
    if (absRealAngleDiff(targetAngle, currentAngle) <= threshold) {
      return a;
    }

    //if (absRealAngleDiff(a, currentAngle) > absRealAngleDiff(b, currentAngle)) {
    if (Math.abs(a - currentAngle) > Math.abs(b - currentAngle)) {
      return b;
    }
    return a;
  }

  /**
   * 
   * Converts targetAngle to equivalent angle in bounds (0, 360]
   * @param targetAngle
   * @return
   */
  public static final double convertAngle0To360(double targetAngle) {
    while (targetAngle < 0) targetAngle += 360;
    return targetAngle % 360.0;
  }

  /**
   * 
   * @param value
   * @param deadzone
   * @return
   */
  public static final double applyDeadzone(double value, double deadzone) {
    if (Math.abs(value) <= deadzone) {
      return 0.0;
    }
    return value;
  }
}
