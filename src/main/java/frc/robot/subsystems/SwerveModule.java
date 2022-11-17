// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.libs.MathPlus;
import frc.robot.libs.SparkMaxController;

public class SwerveModule extends SubsystemBase {
  private final String name;
  private final CANCoder canCoder;

  private final SparkMaxController steerController;
  private final SparkMaxController driveController;

  private SimpleMotorFeedforward steerFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);

  // Tuning
  private Supplier<double[]> steerPIDSupplier;
  private Supplier<double[]> drivePIDSupplier;

  /**
   * 
   * @param steerCANID
   * @param driveCANID
   * @param CANCoderID
   */
  public SwerveModule(
    String name,
    int steerCANID,
    int driveCANID,
    int CANCoderID,
    double magnetOffset
  ) {
    this.name = name;

    steerController = new SparkMaxController(steerCANID, 0, 5);
    driveController = new SparkMaxController(driveCANID, 20, 5);
    canCoder = new CANCoder(CANCoderID);

    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

    // seed internal encoder
    steerController.getSparkMaxEncoder().setPosition(MathPlus.convertAngle0To360(canCoder.getAbsolutePosition() - magnetOffset));
  }
  
  /** */
  public SparkMaxController getSteerController() {
    return steerController;
  }

  /** */
  public SparkMaxController getDriveController() {
    return driveController;
  }

  /** */
  public double getCanCoderAngle() {
    return canCoder.getAbsolutePosition();
  }

  /**
   * Get current angle of module
   * @return
   */
  public double getCurrentAngle() {
    return steerController.getCurrentPosition();
  }

  /**
   * Get current velocity of module
   * @return
   */
  public double getCurrentVelocity() {
    return driveController.getCurrentVelocity();
  }

  public void setSteerFeedforward(SimpleMotorFeedforward ff) {
    steerFeedforward = ff;
  }

  public void setDriveFeedforward(SimpleMotorFeedforward ff) {
    driveFeedforward = ff;
  }

  /** */
  public void setDesiredAngle(double angle) {
    steerController.setDesiredPosition(angle, steerFeedforward.calculate(angle));
  }

  /** */
  public void setDesiredSpeed(double speed) {
    driveController.setDesiredVelocity(speed, driveFeedforward.calculate(speed));
  }

  /**
   * Set desired module state; closed loop
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimizeModuleState(desiredState, steerController.getCurrentPosition());

    setDesiredSpeed(desiredState.speedMetersPerSecond);
    setDesiredAngle(desiredState.angle.getDegrees());
  }

  /**
   * Set desired module state; open loop
   * @param desiredState
   */
  public void setDesiredStateOpenLoop(SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimizeModuleState(desiredState, steerController.getCurrentPosition());

    driveController.set(desiredState.speedMetersPerSecond / SWERVEMODULECONSTANTS.MAX_SPEED);
    setDesiredAngle(desiredState.angle.getDegrees());
  }

  /** */
  public void halt() {
    this.steerController.set(0);
    this.driveController.set(0);
  }

  /** */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
      getCurrentVelocity(),
      Rotation2d.fromDegrees(getCurrentAngle())
    );
  }

  /*
  public SwerveModulePosition getCurrentPositionState() {
    return new SwerveModulePosition(
      driveController.getCurrentPosition(),
      Rotation2d.fromDegrees(getCurrentAngle())
    );
  }
  */

  public void setSteerPIDSupplier(Supplier<double[]> supplier) {
    steerPIDSupplier = supplier;
  }

  public void setDrivePIDSupplier(Supplier<double[]> supplier) {
    drivePIDSupplier = supplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetry == false) return;

    builder.setSmartDashboardType("Swerve Module " + name);
    builder.addDoubleProperty("Velocity", this::getCurrentVelocity, null);
    builder.addDoubleProperty("Angle", this::getCurrentAngle, null);
    builder.addDoubleProperty("CANCoder", this::getCanCoderAngle, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Constants.tuningMode) {
      this.steerController.configPositionControlPID(steerPIDSupplier.get());
      this.driveController.configVelocityControlPID(drivePIDSupplier.get());
    }
  }
  
  /** Less aggressive 120 degree module optimization compared to wpilib 90 deg */
  public static final SwerveModuleState optimizeModuleState(SwerveModuleState state, double currentAngle) {
    return SwerveModule.optimizeModuleState(state, currentAngle, 120.0);
  }

  /**
   * Module state optimization
   * @param state
   * @param currentAngle
   */
  public static final SwerveModuleState optimizeModuleState(SwerveModuleState state, double currentAngle, double threshold) {
    if (MathPlus.absRealAngleDiff(currentAngle, state.angle.getDegrees()) <= threshold) {
      return new SwerveModuleState(
        state.speedMetersPerSecond,
        Rotation2d.fromDegrees(MathPlus.optimizeSwerveAngle(state.angle.getDegrees(), currentAngle, threshold))
      );
    }
    return new SwerveModuleState(
      -state.speedMetersPerSecond,
      Rotation2d.fromDegrees(MathPlus.optimizeSwerveAngle(state.angle.getDegrees() + 180, currentAngle, threshold))
    );
  }
}
