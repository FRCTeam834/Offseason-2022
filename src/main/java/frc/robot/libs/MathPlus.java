package frc.robot.libs;

public class MathPlus {
  /**
   * Is value in the interval [-range, range]
   */
  public static final boolean inclusiveInRange(double value, double range) {
    return (value <= range && value >= -range);
  }
}
