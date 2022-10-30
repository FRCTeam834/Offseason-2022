package frc.robot.libs;

import edu.wpi.first.math.controller.PIDController;

public class PIDGains {
  public double kP;
  public double kI;
  public double kD;
  public double kFF;

  public static PIDGains PD(double kP, double kD) {
    return new PIDGains(kP, 0.0, kD, 0.0);
  }

  public PIDGains(double kP) {
    this(kP, 0.0, 0.0, 0.0);
  }

  public PIDGains(double kP, double kI) {
    this(kP, kI, 0.0, 0.0);
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

  public PIDController generateController() {
    return new PIDController(kP, kI, kD);
  }
}
