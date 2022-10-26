package frc.robot.libs;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;

/**
 * CANSparkMax wrapper
 */
public class CANMotorController {

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController;
  private final RelativeEncoder sparkMaxEncoder;

  private final PIDController velocityPIDController = new PIDController(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward velocityFeedforward = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  private double lastDesiredPosition;
  private double lastDesiredVelocity;

  // Caching
  private PIDState lastPIDState;
  private REVLibError lastError;
  private double lastVoltageOutput;
  private double lastPercentOutput;

  // Custom velocity readings, inspired from 6328
  // https://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/SparkMaxDerivedVelocityController.java
  private final CAN deviceInterface;
  private final LinearFilter velocityFilter; // Rolling derivative
  private final Notifier updateNotifier;

  // https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
  private static final int manufacturerID = 5; // REV
  private static final int deviceTypeID = 2;

  private double lastPosition;
  private double lastVelocity;
  private boolean isFirstPacket = false;

  /**
   * @param deviceID CAN ID
   */
  public CANMotorController(
    int deviceID
  ) {
    this(deviceID, 10, 5);
  }

  /**
   * @param deviceID CAN ID
   * @param updatePeriodMs ms
   * @param filterPoints num points for velocity moving average
   */
  public CANMotorController(
    int deviceID,
    int updatePeriodMs,
    int filterPoints
  ) {
    sparkMax = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
    sparkMaxPIDController = sparkMax.getPIDController();
    sparkMaxEncoder = sparkMax.getEncoder();

    deviceInterface = new CAN(sparkMax.getDeviceId(), manufacturerID, deviceTypeID);
    velocityFilter = LinearFilter.backwardFiniteDifference(1, filterPoints, updatePeriodMs / 1000.0);

    updateNotifier = new Notifier(this::update);
    updateNotifier.startPeriodic(updatePeriodMs / 1000.0);
  }

  /**
   * Stores a SparkMaxPID reference state
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

  /**
   * 
   * Is value in bounds [-range, range]
   * @param value
   * @param range
   * @return
   */
  private static final boolean inclusiveInRange(double value, double range) {
    range = Math.abs(range);
    return (value <= range && value >= -range);
  }

  /** Get sparkmax */
  public CANSparkMax getSparkMax() {
    return this.sparkMax;
  }

  /** Get internal PID controller */
  public SparkMaxPIDController getPIDController() {
    return this.sparkMaxPIDController;
  }

  /** Get internal encoder */
  public RelativeEncoder getEncoder() {
    return this.sparkMaxEncoder;
  }

  /** */
  public void configFactoryDefault() {
    this.sparkMax.restoreFactoryDefaults();
  }
  /** */
  public void configInverted(boolean inverted) {
    this.sparkMax.setInverted(inverted);
  }
  /** */
  public void configIdleMode(CANSparkMax.IdleMode idleMode) {
    this.sparkMax.setIdleMode(idleMode);
  }
  /** */
  public void configVoltageCompensation(double voltage) {
    this.sparkMax.enableVoltageCompensation(voltage);
  }
  /** */
  public void configSmartCurrentLimit(int amps) {
    this.sparkMax.setSmartCurrentLimit(amps);
  }
  /** */
  public void configPeriodicFramePeriods(int k0, int k1, int k2) {
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
  }
  /** */
  public void configVelocityConversionFactor(double factor) {
    this.sparkMaxEncoder.setVelocityConversionFactor(factor);
  }
  /** */
  public void configPositionConversionFactor(double factor) {
    this.sparkMaxEncoder.setPositionConversionFactor(factor);
  }

  /** */
  public void configVelocityFeedforward(double kS, double kV) {
    this.velocityFeedforward = new SimpleMotorFeedforward(kS, kV);
  }
  /** */
  public void configVelocityFeedforward(double kS, double kV, double kA) {
    this.velocityFeedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  /** Set internal PID controller P gain */
  public void configPositionControlP(double kP) {
    this.sparkMaxPIDController.setP(kP, 0);
  }
  /** Set internal PID controller I gain */
  public void configPositionControlI(double kI) {
    this.sparkMaxPIDController.setI(kI, 0);
  }
  /** Set internal PID controller D gain */
  public void configPositionControlD(double kD) {
    this.sparkMaxPIDController.setD(kD, 0);
  }
  /** Set internal PID controller FF gain (corresponds to kV) */
  public void configPositionControlFF(double kFF) {
    this.sparkMaxPIDController.setFF(kFF, 0);
  }

  /** Set internal PID controller P gain */
  public void configVelocityControlP(double kP) {
    this.sparkMaxPIDController.setP(kP, 1);
    this.velocityPIDController.setP(kP);
  }
  /** Set internal PID controller I gain */
  public void configVelocityControlI(double kI) {
    this.sparkMaxPIDController.setI(kI, 1);
    this.velocityPIDController.setI(kI);
  }
  /** Set internal PID controller D gain */
  public void configVelocityControlD(double kD) {
    this.sparkMaxPIDController.setD(kD, 1);
    this.velocityPIDController.setD(kD);
  }
  /** Set internal PID controller FF gain (corresponds to kV) */
  public void configVelocityControlFF(double kFF) {
    this.sparkMaxPIDController.setFF(kFF, 1);
  }

  /** Burn flash */
  public void burnFlash() {
    this.sparkMax.burnFlash();
  }

  /** */
  public boolean isAtDesiredVelocity(double desiredVelocity, double tolerance) {
    return this.isAtDesiredVelocity(desiredVelocity, tolerance, false);
  }

  /** */
  public boolean isAtDesiredPosition(double desiredPosition, double tolerance) {
    return this.isAtDesiredPosition(desiredPosition, tolerance, false);
  }

  /**
   * 
   * @param desiredVelocity
   * @param tolerance bilateral tolerance (+/-)
   * @param useEncoder if true - use encoder calls
   * @return
   */
  public boolean isAtDesiredVelocity(double desiredVelocity, double tolerance, boolean useEncoder) {
    return CANMotorController.inclusiveInRange(this.getCurrentVelocity(useEncoder) - desiredVelocity, tolerance);
  }

  /**
   * 
   * @param desiredPosition
   * @param tolerance bilateral tolerance (+/-)
   * @param useEncoder if true - use encoder calls
   * @return
   */
  public boolean isAtDesiredPosition(double desiredPosition, double tolerance, boolean useEncoder) {
    return CANMotorController.inclusiveInRange(this.getCurrentPosition(useEncoder) - desiredPosition, tolerance);
  }

  /** */
  public double getCurrentVelocity() {
    return this.getCurrentVelocity(false);
  }

  /** */
  public double getCurrentPosition() {
    return this.getCurrentPosition(false);
  }

  /**
   * 
   * @param useEncoder if true - use encoder calls
   * @return
   */
  public synchronized double getCurrentVelocity(boolean useEncoder) {
    if (useEncoder) {
      return this.sparkMaxEncoder.getVelocity();
    }
    return this.lastVelocity;
  }

  /**
   * 
   * @param useEncoder if true - use encoder calls
   * @return
   */
  public synchronized double getCurrentPosition(boolean useEncoder) {
    if (useEncoder) {
      return this.sparkMaxEncoder.getPosition();
    }
    return this.lastPosition;
  }

  /** Reset cache for PID state; Shouldn't be needed */
  public void resetCache() {
    this.lastPIDState = null;
    this.lastError = null;
    this.lastVoltageOutput = Double.NaN;
    this.lastPercentOutput = Double.NaN;
  }

  /** Stop motor */
  public void halt() {
    this.set(0);
  }

  /** */
  public void set(double percent) {
    // PID is killed on any open loop call
    this.lastPIDState = null;

    if (percent == this.lastPercentOutput) return;
    this.lastPercentOutput = percent;

    this.sparkMax.set(percent);
  }
  
  /**
   * Set voltage of motor
   */
  public void setVoltage(double voltage) {
    // PID is killed on any open loop call
    this.lastPIDState = null;

    if (voltage == this.lastVoltageOutput) return;
    this.lastVoltageOutput = voltage;

    this.sparkMax.setVoltage(voltage);
  }

  /** */
  public void setVelocity(double velocity) {
    this.setVelocity(velocity, this.velocityFeedforward);
  }

  /**
   * 
   * @param velocity desired rpm
   * @param feedForward feedforward controller
   */
  public void setVelocity(double velocity, SimpleMotorFeedforward feedForwardController) {
    this.setVoltage(feedForwardController.calculate(velocity));
  }

  /**
   * 
   * @param velocity desired rpm
   * @param kFF feedforward value
   */
  public void setVelocity(double velocity, double kFF) {
    this.setVoltage(velocity * kFF);
    this.configVelocityFeedforward(0.0, kFF);
  }

  /**
   * Set desired velocity of motor
   * @param velocity
   */
  /*
  public REVLibError setDesiredVelocity(double velocity) {
    this.lastVoltageOutput = this.lastPercentOutput = Double.NaN;

    PIDState desiredState = new PIDState(velocity, CANSparkMax.ControlType.kVelocity);
    // Checks if input state is equivalent to the last desired state
    // If so, return as there is nothing to be done
    if (desiredState.equals(this.lastPIDState)) return this.lastError;

    this.lastPIDState = desiredState;
    this.lastDesiredVelocity = velocity;
    return lastError = this.sparkMaxPIDController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 1);
  }
  */
  public void setDesiredVelocity(double velocity) {
    this.lastDesiredVelocity = velocity;
  }

  /**
   * Set desired position of motor
   * @param position
   */
  public REVLibError setDesiredPosition(double position) {
    this.lastVoltageOutput = this.lastPercentOutput = this.lastDesiredVelocity = Double.NaN;

    PIDState desiredState = new PIDState(position, CANSparkMax.ControlType.kPosition);
    // Checks if input state is equivalent to the last desired state
    // If so, return as there is nothing to be done
    if (desiredState.equals(this.lastPIDState)) return this.lastError;

    this.lastPIDState = desiredState;
    this.lastDesiredPosition = position;
    return lastError = this.sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0);
  }

  private void updateVelocityReference() {
    this.setVelocity(this.velocityPIDController.calculate(
      this.getCurrentVelocity(),
      this.lastDesiredVelocity
    ));
  }

  /** */
  private void update() {
    if (!Double.isNaN(this.lastDesiredVelocity)) {
      this.updateVelocityReference();
    }
    // https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMjAvMDUvMTkvMTQvMDYvNDMvNDUyNGFkOTMtZjYwZi00ODgyLWFlNzQtNjAxMzU5MzQyMjBiL2FtLTQyNjEgU1BBUksgTUFYIC0gVXNlciBNYW51YWwuaHRtbCJdXQ/am-4261%20SPARK%20MAX%20-%20User%20Manual.html?sha=7c9ea7a1ed73eb42#section-3-3-2-1
    // Packet contains motor position in rotations
    // as 32-bit (4 byte) IEEE float
    CANData buffer = new CANData();
    this.deviceInterface.readPacketLatest(PeriodicFrame.kStatus2.value, buffer);

    // Extract position data from bytes
    double position = ByteBuffer
      .wrap(buffer.data)
      .order(ByteOrder.BIG_ENDIAN) // CAN data is BE?
      .getFloat();
    // double packetTime = buffer.timestamp;

    if (isFirstPacket) {
      isFirstPacket = false;
      lastPosition = position;
      // Velocity needs 2 data points to be calculated, so return
      return;
    }
    lastVelocity = velocityFilter.calculate(position);
    lastPosition = position;
  }
}
