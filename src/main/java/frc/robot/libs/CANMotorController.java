package frc.robot.libs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * Lightweight CANSparkMax wrapper class with caching
 */
public class CANMotorController {

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController;
  private final RelativeEncoder sparkMaxEncoder;

  // caching; helps reduce load on the CAN bus
  private PIDState lastPIDState;
  private REVLibError lastError;

  // Velocity readings 

  /**
   * @param deviceID CAN ID
   */
  public CANMotorController(
    int deviceID
  ) {
    sparkMax = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    sparkMaxPIDController = sparkMax.getPIDController();
    sparkMaxEncoder = sparkMax.getEncoder();
  }

  /**
   * Stores PID reference state
   */
  private static final class PIDState {
    public final double setpoint;
    public final CANSparkMax.ControlType controlType;

    public PIDState(double setpoint, CANSparkMax.ControlType controlType) {
      this.setpoint = setpoint;
      this.controlType = controlType;
    }

    /** */
    public boolean equals(PIDState expected) {
      if (expected == null) return false;
      return (
        this.setpoint == expected.setpoint &&
        this.controlType.equals(expected.controlType)
      );
    }
  }

  /** */
  public CANSparkMax getSparkMax() {
    return sparkMax;
  }

  /** */
  public SparkMaxPIDController getPIDController() {
    return sparkMaxPIDController;
  }

  /** */
  public RelativeEncoder getEncoder() {
    return sparkMaxEncoder;
  }

  /** */
  public void configFactoryDefault() {
    sparkMax.restoreFactoryDefaults();
  }

  /** */
  public void configIdleMode(CANSparkMax.IdleMode idleMode) {
    sparkMax.setIdleMode(idleMode);
  }

  /** */
  public void configVoltageCompensation(double voltage) {
    sparkMax.enableVoltageCompensation(voltage);
  }

  /** */
  public void configSmartCurrentLimit(int amps) {
    sparkMax.setSmartCurrentLimit(amps);
  }

  /** */
  public void configPeriodicFramePeriods(int k0, int k1, int k2) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
  }

  /** */
  public void setP(double kP) {
    sparkMaxPIDController.setP(kP);
  }
  /** */
  public void setI(double kI) {
    sparkMaxPIDController.setI(kI);
  }
  /** */
  public void setD(double kD) {
    sparkMaxPIDController.setD(kD);
  }

  /** */
  public void burnFlash() {
    sparkMax.burnFlash();
  }

  /** */
  public boolean isAtDesiredVelocity(double desiredVelocity, double tolerance) {
    return MathPlus.inclusiveInRange(this.getCurrentVelocity() - desiredVelocity, tolerance);
  }

  /** */
  public boolean isAtDesiredPosition(double desiredPosition, double tolerance) {
    return MathPlus.inclusiveInRange(this.getCurrentPosition() - desiredPosition, tolerance);
  }

  /** Default rpm (change using conversion factor) */
  public double getCurrentVelocity() {
    return sparkMaxEncoder.getVelocity();
  }

  /** Default n_rot (change using conversion factor) */
  public double getCurrentPosition() {
    return sparkMaxEncoder.getPosition();
  }

  /** Reset cache for PID state; Shouldn't be needed */
  public void resetCache() {
    lastPIDState = null;
    lastError = null;
  }

  /** Stop motor */
  public void halt() {
    this.setVelocity(0);
  }
  
  /**
   * Set velocity of motor
   */
  public void setVelocity(double velocity) {
    // Cache must be reset -- PID is killed on any open loop call
    lastPIDState = null;

    sparkMax.set(velocity);
  }

  /**
   * Set desired velocity of motor
   * @param velocity default: rpm (change using conversion factor)
   */
  public REVLibError setDesiredVelocity(double velocity) {
    PIDState desiredState = new PIDState(velocity, CANSparkMax.ControlType.kVelocity);
    // Checks if input state is equivalent to the last desired state
    // If so, return as there is nothing to be done
    if (desiredState.equals(lastPIDState)) return lastError;

    lastPIDState = desiredState;
    return lastError = sparkMaxPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Set desired position of motor
   * @param position default: n_rot (change using conversion factor)
   */
  public REVLibError setDesiredPosition(double position) {
    PIDState desiredState = new PIDState(position, CANSparkMax.ControlType.kPosition);
    // Checks if input state is equivalent to the last desired state
    // If so, return as there is nothing to be done
    if (desiredState.equals(lastPIDState)) return lastError;

    lastPIDState = desiredState;
    return lastError = sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }
}
