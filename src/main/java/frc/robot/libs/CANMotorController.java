package frc.robot.libs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

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

  private double desiredVelocity = 0.0;
  private boolean atDesiredVelocity = false;

  private double desiredPosition = 0.0;
  private boolean atDesiredPosition = false;

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

  /** Updated per periodic */
  public boolean isAtDesiredVelocity() {
    return atDesiredVelocity;
  }

  /** Updated per periodic */
  public boolean isAtDesiredPosition() {
    return atDesiredPosition;
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

  /** Sets the cached desired velocity */
  private void setCachedDesiredVelocity(double velocity) {
    desiredVelocity = velocity;
    desiredPosition = Double.NaN;
  }

  /** Sets the cached desired position */
  private void setCachedDesiredPosition(double position) {
    desiredPosition = position;
    desiredVelocity = Double.NaN;
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

    setCachedDesiredVelocity(velocity);
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
    setCachedDesiredVelocity(velocity);
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
    setCachedDesiredPosition(position);
    return lastError = sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  /** Call every periodic update */
  public void periodic(
    double atDesiredVelocityTolerance,
    double atDesiredPositionTolerance
  ) {
    // Set if motor is at desired velocity
    this.atDesiredVelocity = MathPlus.inclusiveInRange(
      this.getCurrentVelocity() - this.desiredVelocity,
      atDesiredVelocityTolerance
    );

    // Set if motor is at desired position
    this.atDesiredPosition = MathPlus.inclusiveInRange(
      this.getCurrentPosition() - this.desiredPosition,
      atDesiredPositionTolerance
    );
  }
}
