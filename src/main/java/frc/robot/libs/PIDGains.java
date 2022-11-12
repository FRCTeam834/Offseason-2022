package frc.robot.libs;

import edu.wpi.first.math.controller.PIDController;

/**
 * Helper class that stores PIDF gains
 */
public class PIDGains {
  protected double kP;
  protected double kI;
  protected double kD;
  protected double kFF;

  public PIDGains(double kP) {
    this(kP, 0.0, 0.0, 0.0);
  }

  /**
   * PI controllers are never used, two arg constructor will be a PD controller instead
   */
  public PIDGains(double kP, double kD) {
    this(kP, 0.0, kD, 0.0);
  }

  public PIDGains(double kP, double kI, double kD) {
    this(kP, kI, kD, 0.0);
  }

  public PIDGains(double kP, double kI, double kD, double kFF) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kFF = kFF;
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

  public double getFF() {
    return kFF;
  }

  public double[] getGains() {
    return new double[] { getP(), getI(), getD(), getFF() };
  }

  public PIDController generateController() {
    return new PIDController(kP, kI, kD);
  }
}
