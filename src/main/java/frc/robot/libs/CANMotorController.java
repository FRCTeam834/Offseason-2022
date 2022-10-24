package frc.robot.libs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class CANMotorController {

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController;
  private final RelativeEncoder sparkMaxEncoder;

  // caching
  private PIDState lastPIDState;
  private REVLibError lastError;

  private double desiredVelocity = 0.0;
  private boolean atDesiredVelocity = false;

  private double desiredPosition = 0.0;
  private boolean atDesiredPosition = false;

  /** */
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

  /** */
  public double getCurrentVelocity() {
    return sparkMaxEncoder.getVelocity();
  }

  /** */
  public double getCurrentPosition() {
    return sparkMaxEncoder.getPosition();
  }

  /** */
  private void setCachedDesiredVelocity(double velocity) {
    desiredVelocity = velocity;
    desiredPosition = Double.NaN;
  }

  /** */
  private void setCachedDesiredPosition(double position) {
    desiredPosition = position;
    desiredVelocity = Double.NaN;
  }
  
  /** */
  public void setVelocity(double velocity) {
    // Cache must be reset -- PID is killed on any open loop call
    lastPIDState = null;

    setCachedDesiredVelocity(velocity);
    sparkMax.set(velocity);
  }

  /**
   * @param velocity default: rpm (change using conversion factor)
   */
  public REVLibError setDesiredVelocity(double velocity) {
    PIDState desiredState = new PIDState(velocity, CANSparkMax.ControlType.kVelocity);
    if (desiredState.equals(lastPIDState)) return lastError;

    lastPIDState = desiredState;
    setCachedDesiredVelocity(velocity);
    return lastError = sparkMaxPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * @param position default: n_rot (change using conversion factor)
   */
  public REVLibError setDesiredPosition(double position) {
    PIDState desiredState = new PIDState(position, CANSparkMax.ControlType.kPosition);
    if (desiredState.equals(lastPIDState)) return lastError;

    lastPIDState = desiredState;
    setCachedDesiredPosition(position);
    return lastError = sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition);
  }

  /** */
  public void periodic(
    double atDesiredVelocityTolerance,
    double atDesiredPositionTolerance
  ) {
    // Set if motor is at desired velocity
    this.atDesiredVelocity = MathPlus.inclusiveInRange(
      this.getCurrentVelocity() - this.desiredVelocity,
      atDesiredVelocityTolerance
    );

    this.atDesiredPosition = MathPlus.inclusiveInRange(
      this.getCurrentPosition() - this.desiredPosition,
      atDesiredPositionTolerance
    );
  }
}
