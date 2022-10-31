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
 * Velocity control on rio -- mimics sparkmax api !Note: PIDF gains will be different
 * 
 * @author Keller
 */
public class SparkMaxController {
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
    POSITION, VELOCITY, VOLTAGE, PERCENT
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
  
  /**
   * 
   * 
   * 
   */

  // https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html
  private static final int manufacturerID = 5;
  private static final int deviceTypeID = 2;

  private final CANSparkMax sparkMax;
  private final SparkMaxPIDController sparkMaxPIDController; // PID controller on the spark max
  private final RelativeEncoder sparkMaxEncoder; // NEO internal encoder

  private double lastPosition; // Unit: Rotations
  private double lastVelocity; // Unit: RPM
  private DesiredState lastDesiredState;
  // private double lastDesiredPosition; // Unit: Rotations
  // private double lastDesiredVelocity; // Unit: RPM
  // private double lastDesiredVoltage; // Unit: Volts

  private double velocityConversionFactor;
  private double positionConversionFactor;

  // Custom velocity filtering
  // Inspired by 6328: https://github.com/Mechanical-Advantage/SwerveDevelopment/blob/main/src/main/java/frc/robot/util/SparkMaxDerivedVelocityController.java
  private final CAN deviceInterface; // Read CAN packets directly
  private final LinearFilter velocityFilter; // Rolling derivative
  private final Notifier updateNotifier;
  private boolean isFirstPacket = true;

  // Velocity PIDF
  private PIDController velocityPIDController = new PIDController(0.0, 0.0, 0.0);
  private double velocityFeedforward; // kV
  private double velocityArbFF; // kS

  /**
   * 
   * Creates CANMotorController with 20ms timestep and 5 filter points
   * @param CANID
   */
  public SparkMaxController(int CANID) {
    this(CANID, 20, 5);
  }

  /**
   * 
   * @param CANID CAN ID of sparkmax controller
   * @param updateTimestep milliseconds per update (time delta) !Note: 10ms is the lowest recommended
   * @param velocityFilterPoints number of points on velocity filter
   */
  public SparkMaxController(int CANID, int updateTimestep, int velocityFilterPoints) {
    this.sparkMax = new CANSparkMax(CANID, CANSparkMax.MotorType.kBrushless);
    this.sparkMaxPIDController = this.sparkMax.getPIDController();
    this.sparkMaxEncoder = this.sparkMax.getEncoder();

    this.deviceInterface = new CAN(this.sparkMax.getDeviceId(), SparkMaxController.manufacturerID, SparkMaxController.deviceTypeID);
    this.velocityFilter = LinearFilter.backwardFiniteDifference(1, velocityFilterPoints, updateTimestep / 1000.0);

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
  public SparkMaxController configPeriodicFramePeriods(int k0, int k1, int k2) {
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
    return this;
  }
  public SparkMaxController configPositionConversionFactor(double factor) {
    this.positionConversionFactor = factor;
    return this;
  }
  public SparkMaxController configVelocityConversionFactor(double factor) {
    this.velocityConversionFactor = factor;
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
  public SparkMaxController configPositionControlFF(double kFF) {
    this.sparkMaxPIDController.setFF(kFF, 0);
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
  public SparkMaxController configVelocityControlFF(double kFF) {
    this.velocityFeedforward = kFF;
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
    return this.lastVelocity;
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
    return this.lastPosition;
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

  /** */
  public void set(double percent) {
    DesiredState desiredState = new DesiredState(percent, ControlType.PERCENT);

    if (!lastDesiredState.equals(desiredState)) return;
    this.lastDesiredState = desiredState;

    this.sparkMax.set(percent);
  }

  /**
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    // Caching can be used as sparkMax.setVoltage is a set-and-forget call
    DesiredState desiredState = new DesiredState(voltage, ControlType.VOLTAGE);

    if (!lastDesiredState.equals(desiredState)) return;
    this.lastDesiredState = desiredState;

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
   * Run on RIO - structured to mimic sparkMax integrated PIDF
   * @param velocity
   * @param arbFF
   */
  public void setDesiredVelocity(double velocity, double arbFF) {
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
    position *= this.positionConversionFactor;
    
    DesiredState desiredState = new DesiredState(position, ControlType.POSITION);

    if (!this.lastDesiredState.equals(desiredState)) return;
    this.lastDesiredState = desiredState;

    this.sparkMaxPIDController.setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFF);
  }

  /** */
  private void updateVelocity() {
    if (!this.lastDesiredState.controlType.equals(ControlType.VELOCITY)) return;
    this.setVoltage(
      this.velocityPIDController.calculate(this.lastDesiredState.setpoint, this.getCurrentVelocity()) +
      (this.velocityArbFF * Math.signum(this.lastDesiredState.setpoint) + this.velocityFeedforward * this.lastDesiredState.setpoint) // feedforward calculation ks * signum(vel) + kv * vel
    );
  }

  private void update() {
    this.updateVelocity();
    // https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMjAvMDUvMTkvMTQvMDYvNDMvNDUyNGFkOTMtZjYwZi00ODgyLWFlNzQtNjAxMzU5MzQyMjBiL2FtLTQyNjEgU1BBUksgTUFYIC0gVXNlciBNYW51YWwuaHRtbCJdXQ/am-4261%20SPARK%20MAX%20-%20User%20Manual.html?sha=7c9ea7a1ed73eb42#section-3-3-2-1
    // Packet contains motor position in rotations
    // as 32-bit (4 byte) IEEE float
    CANData buffer = new CANData();
    this.deviceInterface.readPacketLatest(PeriodicFrame.kStatus2.value, buffer);

    // Extract position data from bytes
    double position = ByteBuffer
      .wrap(buffer.data)
      .order(ByteOrder.LITTLE_ENDIAN)
      .getFloat();
    // double packetTime = buffer.timestamp;
    lastPosition = position;

    if (isFirstPacket) {
      isFirstPacket = false;
      // Velocity needs 2 data points to be calculated, so return
      return;
    }
    lastVelocity = velocityFilter.calculate(position);
  }
}
