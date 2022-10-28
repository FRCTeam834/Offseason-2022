// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.libs.SparkMaxController;
import frc.robot.libs.MathPlus;

public class SwerveModule extends SubsystemBase {

  private final SparkMaxController steerController;
  private final SparkMaxController driveController;
  private final CANCoder canCoder;
  private PIDController drivePIDController;
  private SimpleMotorFeedforward driveFeedforward;

  // caching
  private SwerveModuleState lastDesiredState = new SwerveModuleState();

  /** Creates a new SwerveModule. */
  public SwerveModule(
    int steerID,
    int driveID,
    int CANCoderID,
    double ffkS,
    double ffkV,
    double ffkA
  ) {
    steerController = new SparkMaxController(steerID);
    driveController = new SparkMaxController(driveID);
    canCoder = new CANCoder(CANCoderID);

    drivePIDController = new PIDController(0.0, 0.0, 0.0);
    driveFeedforward = new SimpleMotorFeedforward(ffkS, ffkV, ffkA);

    // Config
    // These should not be touched (hence not in Constants)
    steerController.configRestoreFactoryDefaults();
    steerController.configInverted(false);
    steerController.configIdleMode(CANSparkMax.IdleMode.kBrake);
    steerController.configVoltageCompensation(12.0);
    steerController.configSmartCurrentLimit(20);
    steerController.configPeriodicFramePeriods(10, 20, 10);

    driveController.configRestoreFactoryDefaults();
    driveController.configInverted(false);
    driveController.configIdleMode(CANSparkMax.IdleMode.kBrake);
    driveController.configVoltageCompensation(12.0);
    driveController.configSmartCurrentLimit(20);
    driveController.configPeriodicFramePeriods(10, 20, 10);

    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
  }

  /**
   * 
   * Less aggressive swerve state optimization using 120 deg threshold instead of 90
   * @param state
   * @param currentAngle current angle of module in deg
   * @return
   */
  public static final SwerveModuleState optimizeState(
    SwerveModuleState state,
    double currentAngle
  ) {
    double angle = state.angle.getDegrees();
    double velocity = state.speedMetersPerSecond;

    // Turn distance needed less than 120 degrees, no optimization needed
    if (MathPlus.absRealAngleDiff(angle, currentAngle) <= 120) return state;

    // Optimized state
    return new SwerveModuleState(
      // Reverse drive direction
      -velocity,
      // Opposite angle
      Rotation2d.fromDegrees((angle - 180) % 360.0)
    );
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
  public void setSteerP(double kP) {
    steerController.configPositionControlP(kP);
  }
  /** */
  public void setSteerI(double kI) {
    steerController.configPositionControlI(kI);
  }
  /** */
  public void setSteerD(double kD) {
    steerController.configPositionControlD(kD);
  }
  /** */
  public void setDriveP(double kP) {
    // driveController.setP(kP);
    driveController.configVelocityControlP(kP);
  }
  /** */
  public void setDriveI(double kI) {
    // driveController.setI(kI);
    driveController.configVelocityControlI(kI);
  }
  /** */
  public void setDriveD(double kD) {
    // driveController.setP(kD);
    driveController.configVelocityControlD(kD);
  }

  /** */
  public double getCurrentAngle() {
    return canCoder.getAbsolutePosition();
  }

  /** Default: rpm (change using conversion factor) */
  public void setDesiredVelocity(double velocity) {
    lastDesiredState.speedMetersPerSecond = velocity;
    driveController.setDesiredVelocity(velocity);
  }

  /** Default: rot (change using conversion factor) */
  public void setDesiredAngle(double angle) {
    lastDesiredState.angle = Rotation2d.fromDegrees(angle);
    steerController.setDesiredPosition(angle);
  }

  /** */
  public void setDesiredState(SwerveModuleState desiredState) {
    // RIP pure functions :P
    desiredState = SwerveModule.optimizeState(desiredState, this.getCurrentAngle());

    this.setDesiredAngle(desiredState.angle.getDegrees());
    this.setDesiredVelocity(desiredState.speedMetersPerSecond);
    // this.setDesiredVelocity(desiredState.speedMetersPerSecond);
  }

  /** */
  public boolean isAtVelocity(double velocity) {
    return driveController.isAtVelocity(velocity, SwerveModuleConstants.AT_VELOCITY_TOLERANCE);
  }

  /** */
  public boolean isAtPosition(double position) {
    return steerController.isAtPosition(position, SwerveModuleConstants.AT_POSITION_TOLERANCE);
  }

  @Override
  public void periodic() {
    // Update relative encoder with CANCoder angle
    // steerController.getEncoder().setPosition(this.getCurrentAngle());
  }
}
