package frc.robot.libs;

import edu.wpi.first.math.controller.PIDController;

/**
 * Helper class that stores PID gains
 */
public class PIDGains {
  protected double kP;
  protected double kI;
  protected double kD;

  public PIDGains(double kP) {
    this(kP, 0.0, 0.0);
  }

  /**
   * PI controllers are never used, two arg constructor will be a PD controller instead
   */
  public PIDGains(double kP, double kD) {
    this(kP, 0.0, kD);
  }

  public PIDGains(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  public double getP() {
    return kP;
  }

  public double getI() {
    return kI;
  }

  public double getD() {
    return kD;
  }

  public double[] getGains() {
    return new double[] { getP(), getI(), getD() };
  }

  public boolean hasChanged() { return false; }

  public PIDController generateController() {
    return new PIDController(kP, kI, kD);
  }
}
