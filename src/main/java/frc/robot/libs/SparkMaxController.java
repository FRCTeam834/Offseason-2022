package frc.robot.libs;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Notifier;

/**
 * 
 * CANSparkMax wrapper class
 * 
 * Positional control on spark maxes
 * Velocity control on rio -- mimics sparkmax api !Note: PID gains will be different
 * 
 * @author Keller
 */
public class SparkMaxController {
  // https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
  private static final int manufacturerID = 5;
  private static final int deviceTypeID = 2;
  private static final int apiIdStatus2 = 98;

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController; // PID controller on the spark max
  private final RelativeEncoder sparkMaxEncoder; // NEO internal encoder

  private double lastPosition = 0.0; // Rotations
  private double lastVelocity = 0.0; // RPM
  private long lastPacketTime = 0; // ms
  private DesiredState lastDesiredState = new DesiredState(0.0, ControlType.PERCENT);
  // private double lastDesiredPosition; // Unit: Rotations
  // private double lastDesiredVelocity; // Unit: RPM
  // private double lastDesiredVoltage; // Unit: Volts

  private double velocityConversionFactor;
  private double positionConversionFactor;

  // Velocity PID
  private PIDController velocityPIDController = new PIDController(0.0, 0.0, 0.0);
  private double velocityArbFF;

  // Position PID
  private PIDController positionPIDController = new PIDController(0.0, 0.0, 0.0);
  private double positionArbFF;

  // Custom velocity filtering
  // Inspired by 6328: https://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/SparkMaxDerivedVelocityController.java
  private final CAN deviceInterface; // Read CAN packets directly
  private final LinearFilter velocityFilter; // Rolling derivative
  private final Notifier updateNotifier;

  // Stall detection 
  // !Important: A low k1 frame period is needed for accurate current readings
  private LinearFilter currentFilter;
  private double lastCurrent; // amps

  /**
   * 
   * Creates CANMotorController with 20ms timestep and 5 filter points
   * @param CANID
   */
  public SparkMaxController(int CANID) {
    this(CANID, 20, 5, 5);
  }

  /** */
  public SparkMaxController(int CANID, int updateTimestep, int velocityFilterPoints) {
    this(CANID, updateTimestep, velocityFilterPoints, 5);
  }

  /**
   * 
   * @param CANID CAN ID of sparkmax controller
   * @param updateTimestep milliseconds per update (time delta) !Note: 10ms is the lowest recommended
   * @param velocityFilterPoints number of points on velocity filter
   * @param currentFilterPoints number of points on current filter
   */
  public SparkMaxController(int CANID, int updateTimestep, int velocityFilterPoints, int currentFilterPoints) {
    this.sparkMax = new CANSparkMax(CANID, CANSparkMax.MotorType.kBrushless);
    this.sparkMaxPIDController = this.sparkMax.getPIDController();
    this.sparkMaxEncoder = this.sparkMax.getEncoder();

    this.deviceInterface = new CAN(this.sparkMax.getDeviceId(), SparkMaxController.manufacturerID, SparkMaxController.deviceTypeID);
    // !Note: backwardFiniteDifference is too noisy to be effective
    // this.velocityFilter = LinearFilter.backwardFiniteDifference(1, velocityFilterPoints, updateTimestep / 1000.0);
    this.velocityFilter = LinearFilter.movingAverage(velocityFilterPoints);
    this.currentFilter = LinearFilter.movingAverage(currentFilterPoints);

    this.updateNotifier = new Notifier(this::update);
    if (updateTimestep > 0) {
      this.updateNotifier.startPeriodic(updateTimestep / 1000.0);
    }
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
    this.positionConversionFactor = factor;
    this.sparkMaxEncoder.setPositionConversionFactor(factor);
    return this;
  }
  public SparkMaxController configVelocityConversionFactor(double factor) {
    this.velocityConversionFactor = factor;
    this.sparkMaxEncoder.setVelocityConversionFactor(factor);
    return this;
  }

  public SparkMaxController configPositionControlP(double kP) {
    this.positionPIDController.setP(kP);
    return this;
  }
  public SparkMaxController configPositionControlI(double kI) {
    this.positionPIDController.setI(kI);
    return this;
  }
  public SparkMaxController configPositionControlD(double kD) {
    this.positionPIDController.setD(kD);
    return this;
  }
  public SparkMaxController configPositionControlPID(double[] gains) {
    configPositionControlP(gains[0]);
    configPositionControlI(gains[1]);
    configPositionControlD(gains[2]);
    return this;
  }

  public SparkMaxController configVelocityControlP(double kP) {
    this.velocityPIDController.setP(kP);
    return this;
  }
  public SparkMaxController configVelocityControlI(double kI) {
    this.velocityPIDController.setI(kI);
    return this;
  }
  public SparkMaxController configVelocityControlD(double kD) {
    this.velocityPIDController.setD(kD);
    return this;
  }
  public SparkMaxController configVelocityControlPID(double[] gains) {
    configVelocityControlP(gains[0]);
    configVelocityControlI(gains[1]);
    configVelocityControlD(gains[2]);
    return this;
  }

  public SparkMaxController burnFlash() {
    this.sparkMax.burnFlash();
    return this;
  }

  //

  /** */
  public double getCurrentVelocity() {
    return this.getCurrentVelocity(false);
  }

  /**
   * 
   * @param useEncoder if true - get velocity from encoder !Not recommended
   * @return
   */
  public synchronized double getCurrentVelocity(boolean useEncoder) {
    if (useEncoder) {
      return this.sparkMaxEncoder.getVelocity();
    }
    return this.lastVelocity * this.velocityConversionFactor;
  }

  /** */
  public double getCurrentPosition() {
    return this.getCurrentPosition(false);
  }

  /**
   * 
   * @param useEncoder if true - get position from encoder
   * @return
   */
  public synchronized double getCurrentPosition(boolean useEncoder) {
    if (useEncoder) {
      return this.sparkMaxEncoder.getPosition();
    }
    return this.lastPosition * this.positionConversionFactor;
  }

  /** */
  public boolean isAtPosition(double position, double tolerance) {
    return this.isAtPosition(position, tolerance, false);
  }

  /**
   * 
   * @param position target position
   * @param tolerance bilateral tolerance (+/-)
   * @param useEncoder if true - get position from encoder
   * @return
   */
  public boolean isAtPosition(double position, double tolerance, boolean useEncoder) {
    return SparkMaxController.inInclusiveRange(this.getCurrentPosition(useEncoder) - position, tolerance);
  }

  /** */
  public boolean isAtVelocity(double velocity, double tolerance) {
    return this.isAtVelocity(velocity, tolerance, false);
  }

  /**
   * 
   * @param velocity target velocity
   * @param tolerance bilateral tolerance (+/-)
   * @param useEncoder if true - get velocity from encoder !Not recommended
   * @return
   */
  public boolean isAtVelocity(double velocity, double tolerance, boolean useEncoder) {
    return SparkMaxController.inInclusiveRange(this.getCurrentVelocity(useEncoder) - velocity, tolerance);
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
    DesiredState desiredState = new DesiredState(percent, ControlType.PERCENT);

    if (lastDesiredState.equals(desiredState)) return;
    this.lastDesiredState = desiredState;

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
    // Apply conversion factor
    velocity *= this.velocityConversionFactor;

    this.lastDesiredState = new DesiredState(velocity, ControlType.VELOCITY);
    this.velocityArbFF = arbFF;
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
    // Apply conversion factor
    position *= this.positionConversionFactor;

    this.lastDesiredState = new DesiredState(position, ControlType.POSITION);
    this.positionArbFF = arbFF;
  }

  /** */
  private void updateVelocity() {
    if (!this.lastDesiredState.controlType.equals(ControlType.VELOCITY)) return;

    double desiredVelocity = this.lastDesiredState.setpoint;
    this.setVoltage(
      this.velocityPIDController.calculate(desiredVelocity, this.getCurrentVelocity())
      + this.velocityArbFF
    );
  }

  /** */
  private void updatePosition() {
    if (!this.lastDesiredState.controlType.equals(ControlType.POSITION)) return;

    double desiredPosition = this.lastDesiredState.setpoint;
    this.setVoltage(
      this.positionPIDController.calculate(this.getCurrentPosition() - desiredPosition, 0)
      + this.positionArbFF
    );
  }

  private void update() {
    this.lastCurrent = this.currentFilter.calculate(this.sparkMax.getOutputCurrent());

    // https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMjAvMDUvMTkvMTQvMDYvNDMvNDUyNGFkOTMtZjYwZi00ODgyLWFlNzQtNjAxMzU5MzQyMjBiL2FtLTQyNjEgU1BBUksgTUFYIC0gVXNlciBNYW51YWwuaHRtbCJdXQ/am-4261%20SPARK%20MAX%20-%20User%20Manual.html?sha=7c9ea7a1ed73eb42#section-3-3-2-1
    // Packet contains motor position in rotations
    // as 32-bit (4 byte) IEEE float
    CANData buffer = new CANData();
    this.deviceInterface.readPacketLatest(apiIdStatus2, buffer);

    // Extract position data from bytes
    double position = ByteBuffer
      .wrap(buffer.data)
      .order(ByteOrder.LITTLE_ENDIAN)
      .getFloat();

    long packetTime = buffer.timestamp;

    double velocity = (position - lastPosition) / (double)(packetTime - lastPacketTime);

    // First packet, return since velocity calculation can't be done with only 1 data point
    if (lastPacketTime == 0) {
      lastPacketTime = packetTime;
      return;
    }

    lastPosition = position;
    lastPacketTime = packetTime;
    lastVelocity = velocityFilter.calculate(velocity);

    this.updateVelocity();
    this.updatePosition();
  }

  /** */

  /**
   * Stores a SparkMaxPIDController reference state
   * (Replaced by DesiredState)
   */
  @Deprecated (forRemoval = true)
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

  /** */
  private enum ControlType {
    POSITION, VELOCITY, PERCENT
  }

  /**
   * Stores the last desired state
   */
  private static final class DesiredState {
    public final double setpoint;
    public final ControlType controlType;

    public DesiredState(double setpoint, ControlType controlType) {
      this.setpoint = setpoint;
      this.controlType = controlType;
    }

    public boolean equals(DesiredState expected) {
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
