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
  /**
   * 
   * Less aggressive state optimization using 120 degrees instead of 90
   * @param state
   * @param currentAngle
   */
  public static final SwerveModuleState optimizeModuleState(SwerveModuleState state, double currentAngle) {
    if (MathPlus.absRealAngleDiff(currentAngle, state.angle.getDegrees()) < 120) {
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
  private final double magnetOffset;

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
    this.magnetOffset = magnetOffset;

    canCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    // Seed relative encoder on startup
    steerController.getSparkMaxEncoder().setPosition(canCoder.getAbsolutePosition());
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
    return canCoder.getAbsolutePosition() - magnetOffset;
  }

  /**
   * Set desired module state; closed loop
   * @param desiredState
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimizeModuleState(desiredState, steerController.getCurrentPosition());

    driveController.setDesiredVelocity(desiredState.speedMetersPerSecond);
    steerController.setDesiredPosition(desiredState.angle.getDegrees());
  }

  /**
   * Set desired module state; open loop
   * @param desiredState
   */
  public void setDesiredStateOpenLoop(SwerveModuleState desiredState) {
    desiredState = SwerveModule.optimizeModuleState(desiredState, steerController.getCurrentPosition());

    driveController.getSparkMax().set(desiredState.speedMetersPerSecond / SWERVEMODULECONSTANTS.MAX_SPEED);
    steerController.setDesiredPosition(desiredState.angle.getDegrees());
  }

  /** */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(
      driveController.getCurrentVelocity(),
      Rotation2d.fromDegrees(steerController.getCurrentPosition())
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
