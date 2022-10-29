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
   * @param a deg
   * @param b deg
   * @return deg
   */
  public static final double realAngleDiff(double a, double b) {
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
  public static final double absRealAngleDiff(double a, double b) {
    return Math.abs(realAngleDiff(a, b));
  }

  /**
   * 
   * Matches targedAngle to scope of scopeAngle (0, 360]
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
}
