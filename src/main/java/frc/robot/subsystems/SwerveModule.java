// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.libs.MathPlus;
import frc.robot.libs.SparkMaxController;

public class SwerveModule extends SubsystemBase {

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
    if (MathPlus.absRealAngleDiff(currentAngle, state.angle.getDegrees()) < threshold) {
      return new SwerveModuleState(
        state.speedMetersPerSecond,
        Rotation2d.fromDegrees(MathPlus.matchAngleScope(state.angle.getDegrees(), currentAngle))
      );
    }
    return new SwerveModuleState(
      -state.speedMetersPerSecond,
      Rotation2d.fromDegrees(MathPlus.matchAngleScope(state.angle.getDegrees() + 180, currentAngle))
    );
  }

  private final SparkMaxController steerController;
  private final SparkMaxController driveController;
  private final CANCoder canCoder;

  private SwerveModuleState desiredState;
  private boolean openLoop;

  /**
   * 
   * @param steerCANID
   * @param driveCANID
   * @param CANCoderID
   */
  public SwerveModule(
    int steerCANID,
    int driveCANID,
    int CANCoderID,
    double magnetOffset
  ) {
    steerController = new SparkMaxController(steerCANID, 0, 0);
    driveController = new SparkMaxController(steerCANID, 20, 5);
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

  /**
   * Set desired module state
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean openLoop) {
    desiredState = SwerveModule.optimizeModuleState(desiredState, steerController.getCurrentPosition());
    this.desiredState = desiredState;
    this.openLoop = openLoop;
  }

  /** */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
      getCurrentVelocity(),
      Rotation2d.fromDegrees(getCurrentAngle())
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (this.desiredState == null) return;
    
    if (openLoop) {
      driveController.getSparkMax().set(this.desiredState.speedMetersPerSecond / SWERVEMODULECONSTANTS.MAX_SPEED);
      steerController.setDesiredPosition(this.desiredState.angle.getDegrees());
    } else {
      driveController.setDesiredVelocity(this.desiredState.speedMetersPerSecond);
      steerController.setDesiredPosition(this.desiredState.angle.getDegrees());
    }
  }
}
