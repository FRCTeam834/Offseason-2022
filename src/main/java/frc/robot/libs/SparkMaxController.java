package frc.robot.libs;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.Notifier;

/**
 * 
 * CANSparkMax wrapper class
 * 
 */
public class SparkMaxController {
  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController; // PID controller on the spark max
  private final RelativeEncoder sparkMaxEncoder; // NEO integrated encoder

  private SparkMaxPIDState lastDesiredState = new SparkMaxPIDState(0.0, CANSparkMax.ControlType.kDutyCycle);

  // Stall detection 
  // !Important: A low k1 frame period is needed for accurate current readings
  private LinearFilter currentFilter;
  private double lastCurrent = 0.0; // amps
  
  private Notifier updateNotifier = null;

  /**
   * 
   * @param CANID CAN ID of sparkmax controller
   * 
   */
  public SparkMaxController(int CANID) {
    this(CANID, false);
  }

  /** */
  public SparkMaxController(int CANID, boolean enableStallDetection) {
    this.sparkMax = new CANSparkMax(CANID, CANSparkMax.MotorType.kBrushless);
    this.sparkMaxPIDController = this.sparkMax.getPIDController();
    this.sparkMaxEncoder = this.sparkMax.getEncoder();

    // 5 taps 20ms period hard coded rn
    if (enableStallDetection) {
      this.currentFilter = LinearFilter.movingAverage(5);
      updateNotifier = new Notifier(this::stallDetectionUpdate);
      updateNotifier.startPeriodic(20.0 / 1000.0);
    }
  }

  private void stallDetectionUpdate() {
    this.lastCurrent = this.currentFilter.calculate(this.sparkMax.getOutputCurrent());
  }

  public CANSparkMax getSparkMax() {
    return this.sparkMax;
  }
  public SparkMaxPIDController getSparkMaxPIDController() {
    return this.sparkMaxPIDController;
  }
  public RelativeEncoder getSparkMaxEncoder() {
    return this.sparkMaxEncoder;
  }

  // Config methods

  public SparkMaxController configRestoreFactoryDefaults() {
    this.sparkMax.restoreFactoryDefaults();
    return this;
  }
  public SparkMaxController configInverted(boolean isInverted) {
    this.sparkMax.setInverted(isInverted);
    return this;
  }
  public SparkMaxController configIdleMode(CANSparkMax.IdleMode idleMode) {
    this.sparkMax.setIdleMode(idleMode);
    return this;
  }
  public SparkMaxController configVoltageCompensation(double voltage) {
    this.sparkMax.enableVoltageCompensation(voltage);
    return this;
  }
  public SparkMaxController configSmartCurrentLimit(int limit) {
    this.sparkMax.setSmartCurrentLimit(limit);
    return this;
  }
  // Periodic Frame Documentation
  // https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMjAvMDUvMTkvMTQvMDYvNDMvNDUyNGFkOTMtZjYwZi00ODgyLWFlNzQtNjAxMzU5MzQyMjBiL2FtLTQyNjEgU1BBUksgTUFYIC0gVXNlciBNYW51YWwuaHRtbCJdXQ/am-4261%20SPARK%20MAX%20-%20User%20Manual.html?sha=7c9ea7a1ed73eb42#section-3-3-2-1
  // https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
  public SparkMaxController configPeriodicFramePeriods(int k0, int k1, int k2) {
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
    return this;
  }
  public SparkMaxController configPeriodicFramePeriods(int k0, int k1, int k2, int k3) {
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, k3);
    return this;
  }
  public SparkMaxController configControlFramePeriod(int ms) {
    this.sparkMax.setControlFramePeriodMs(ms);
    return this;
  }
  public SparkMaxController configPositionConversionFactor(double factor) {
    this.sparkMaxEncoder.setPositionConversionFactor(factor);
    return this;
  }
  public SparkMaxController configVelocityConversionFactor(double factor) {
    this.sparkMaxEncoder.setVelocityConversionFactor(factor);
    return this;
  }

  public SparkMaxController configPositionControlP(double kP) {
    this.sparkMaxPIDController.setP(kP, 0);
    return this;
  }
  public SparkMaxController configPositionControlI(double kI) {
    this.sparkMaxPIDController.setI(kI, 0);
    return this;
  }
  public SparkMaxController configPositionControlD(double kD) {
    this.sparkMaxPIDController.setD(kD, 0);
    return this;
  }
  public SparkMaxController configPositionControlPID(double[] gains) {
    configPositionControlP(gains[0]);
    configPositionControlI(gains[1]);
    configPositionControlD(gains[2]);
    return this;
  }

  public SparkMaxController configVelocityControlP(double kP) {
    this.sparkMaxPIDController.setP(kP, 1);
    return this;
  }
  public SparkMaxController configVelocityControlI(double kI) {
    this.sparkMaxPIDController.setI(kI, 1);
    return this;
  }
  public SparkMaxController configVelocityControlD(double kD) {
    this.sparkMaxPIDController.setD(kD, 1);
    return this;
  }
  public SparkMaxController configVelocityControlPID(double[] gains) {
    configVelocityControlP(gains[0]);
    configVelocityControlI(gains[1]);
    configVelocityControlD(gains[2]);
    return this;
  }
  public SparkMaxController configVelocityFilter() {
    // COMING SOON... REV DON'T DISAPPOINT <3
    return this;
  }

  public SparkMaxController burnFlash() {
    this.sparkMax.burnFlash();
    return this;
  }

  //

  /** */
  public synchronized double getCurrentVelocity() {
    return this.sparkMaxEncoder.getVelocity();
  }

  /** */
  public synchronized double getCurrentPosition() {
    return this.sparkMaxEncoder.getPosition();
  }

  /**
   * 
   * @param position target position
   * @param tolerance bilateral tolerance (+/-)
   * @return
   */
  public boolean isAtPosition(double position, double tolerance) {
    return SparkMaxController.inInclusiveRange(this.getCurrentPosition() - position, tolerance);
  }

  /**
   * 
   * @param velocity target velocity
   * @param tolerance bilateral tolerance (+/-)
   * @return
   */
  public boolean isAtVelocity(double velocity, double tolerance) {
    return SparkMaxController.inInclusiveRange(this.getCurrentVelocity() - velocity, tolerance);
  }

  /**
   * 
   * Returns if motor is current is above stall current
   * @param stallAmps amps considered to be stalling
   * @return
   */
  public boolean isStalling(int stallAmps) {
    return this.lastCurrent >= stallAmps;
  }

  /** */
  public void set(double percent) {
    this.sparkMax.set(percent);
  }

  /**
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    this.sparkMax.setVoltage(voltage);
  }

  /**
   * 
   * @param velocity
   */
  public void setDesiredVelocity(double velocity) {
    this.setDesiredVelocity(velocity, 0.0);
  }

  /**
   * 
   * @param velocity
   * @param arbFF - feedforward value
   */
  public void setDesiredVelocity(double velocity, double arbFF) {
    SparkMaxPIDState desiredState = new SparkMaxPIDState(velocity, CANSparkMax.ControlType.kVelocity);
    if (this.lastDesiredState.equals(desiredState)) return;

    this.lastDesiredState = desiredState;
    this.sparkMaxPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 1, arbFF);
  }

  /** */
  public void setDesiredPosition(double position) {
    this.setDesiredPosition(position, 0.0);
  }

  /**
   * 
   * @param position
   * @param arbFF
   * @return void
   */
  public void setDesiredPosition(double position, double arbFF) {
    SparkMaxPIDState desiredState = new SparkMaxPIDState(position, CANSparkMax.ControlType.kPosition);
    if (this.lastDesiredState.equals(desiredState)) return;

    this.lastDesiredState = desiredState;
    this.sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 1, arbFF);
  }

  /**
   * Stores a SparkMaxPIDController reference state
   */
  private static final class SparkMaxPIDState {
    public final double setpoint;
    public final CANSparkMax.ControlType controlType;

    public SparkMaxPIDState(double setpoint, CANSparkMax.ControlType controlType) {
      this.setpoint = setpoint;
      this.controlType = controlType;
    }

    public boolean equals(SparkMaxPIDState expected) {
      if (expected == null) return false;
      return (
        this.setpoint == expected.setpoint &&
        this.controlType.equals(expected.controlType)
      );
    }
  }

  /**
   * 
   * Is value in the given interval [-range, range]
   * @param value tested value
   * @param range interval bounds
   * @return
   */
  private static final boolean inInclusiveRange(double value, double range) {
    return (
      value <= Math.abs(range) &&
      value >= -Math.abs(range)
    );
  }
}
