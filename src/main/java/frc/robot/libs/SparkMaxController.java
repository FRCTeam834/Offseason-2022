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
 * Velocity control on rio -- mimics sparkmax api
 */
public class SparkMaxController {
  /**
   * Stores a SparkMaxPIDController reference state
   * (Replaced by this.lastDesired<ControlType>)
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
  private double lastDesiredPosition; // Unit: Rotations
  private double lastDesiredVelocity; // Unit: RPM
  private double lastDesiredVoltage; // Unit: Volts

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
   * Creates CANMotorController with 10ms timestep and 5 filter points
   * @param CANID
   */
  public SparkMaxController(int CANID) {
    this(CANID, 10, 5);
  }

  /**
   * 
   * @param CANID CAN ID of sparkmax controller
   * @param updateTimestep milliseconds per update (time delta)
   * @param velocityFilterPoints number of points on velocity filter
   */
  public SparkMaxController(int CANID, int updateTimestep, int velocityFilterPoints) {
    this.sparkMax = new CANSparkMax(CANID, CANSparkMax.MotorType.kBrushless);
    this.sparkMaxPIDController = this.sparkMax.getPIDController();
    this.sparkMaxEncoder = this.sparkMax.getEncoder();

    this.deviceInterface = new CAN(this.sparkMax.getDeviceId(), SparkMaxController.manufacturerID, SparkMaxController.deviceTypeID);
    this.velocityFilter = LinearFilter.backwardFiniteDifference(1, velocityFilterPoints, updateTimestep / 1000.0);

    this.updateNotifier = new Notifier(this::update);
    this.updateNotifier.startPeriodic(updateTimestep / 1000.0);
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

  public void configRestoreFactoryDefaults() {
    this.sparkMax.restoreFactoryDefaults();
  }
  public void configInverted(boolean isInverted) {
    this.sparkMax.setInverted(isInverted);
  }
  public void configIdleMode(CANSparkMax.IdleMode idleMode) {
    this.sparkMax.setIdleMode(idleMode);
  }
  public void configVoltageCompensation(double voltage) {
    this.sparkMax.enableVoltageCompensation(voltage);
  }
  public void configSmartCurrentLimit(int limit) {
    this.sparkMax.setSmartCurrentLimit(limit);
  }
  public void configPeriodicFramePeriods(int k0, int k1, int k2) {
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, k0);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, k1);
    this.sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, k2);
  }
  public void configPositionConversionFactor(double factor) {
    this.positionConversionFactor = factor;
  }
  public void configVelocityConversionFactor(double factor) {
    this.velocityConversionFactor = factor;
  }

  public void configPositionControlP(double kP) {
    this.sparkMaxPIDController.setP(kP, 0);
  }
  public void configPositionControlI(double kI) {
    this.sparkMaxPIDController.setI(kI, 0);
  }
  public void configPositionControlD(double kD) {
    this.sparkMaxPIDController.setD(kD, 0);
  }
  public void configPositionControlFF(double kFF) {
    this.sparkMaxPIDController.setFF(kFF, 0);
  }

  public void configVelocityControlP(double kP) {
    this.velocityPIDController.setP(kP);
  }
  public void configVelocityControlI(double kI) {
    this.velocityPIDController.setI(kI);
  }
  public void configVelocityControlD(double kD) {
    this.velocityPIDController.setD(kD);
  }
  public void configVelocityControlFF(double kFF) {
    this.velocityFeedforward = kFF;
  }

  public void burnFlash() {
    this.sparkMax.burnFlash();
  }

  //

  /** Middleman through all position control */
  private void setLastDesiredPosition(double position) {
    this.lastDesiredVelocity = Double.NaN;
    this.lastDesiredPosition = position * this.positionConversionFactor;
  }

  /** Middleman through all velocity control */
  private void setLastDesiredVelocity(double velocity) {
    this.lastDesiredPosition = Double.NaN;
    this.lastDesiredVelocity = velocity * this.velocityConversionFactor;
  }

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

  /**
   * 
   * @param voltage
   */
  public void setVoltage(double voltage) {
    // Caching can be used as sparkMax.setVoltage is a set-and-forget call
    if (voltage == this.lastDesiredVoltage) return;
    this.lastDesiredVoltage = voltage;

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
    this.setLastDesiredVelocity(velocity);
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
    if (position == this.lastDesiredPosition) return;

    this.setLastDesiredPosition(position);
    this.sparkMaxPIDController.setReference(this.lastDesiredPosition, CANSparkMax.ControlType.kPosition, 0, arbFF);
  }

  /** */
  private void updateVelocity() {
    if (Double.isNaN(this.lastDesiredVelocity)) return;
    this.setVoltage(
      this.velocityPIDController.calculate(this.lastDesiredVelocity, this.getCurrentVelocity()) +
      (this.velocityArbFF * Math.signum(this.lastDesiredVelocity) + this.velocityFeedforward * this.lastDesiredVelocity) // feedforward calculation ks * signum(vel) + kv * vel
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
      .order(ByteOrder.BIG_ENDIAN)
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
