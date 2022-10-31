// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAINCONSTANTS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.utilities.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
  private final Pigeon gyro;

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain(Pigeon gyro) {
    this.gyro = gyro;

    frontLeftModule = SwerveModuleFactory.getFLModule();
    frontRightModule = SwerveModuleFactory.getFRModule();
    backLeftModule = SwerveModuleFactory.getBLModule();
    backRightModule = SwerveModuleFactory.getBRModule();

    kinematics = new SwerveDriveKinematics(
      DRIVETRAINCONSTANTS.FLM_POS,
      DRIVETRAINCONSTANTS.FRM_POS,
      DRIVETRAINCONSTANTS.BLM_POS,
      DRIVETRAINCONSTANTS.BRM_POS
    );

    odometry = new SwerveDriveOdometry(kinematics, gyro.getYawAsRotation2d());
  }

  private SwerveModuleState[] getCurrentModuleStates() {
    return new SwerveModuleState[] {
      frontLeftModule.getCurrentState(),
      frontRightModule.getCurrentState(),
      backLeftModule.getCurrentState(),
      backRightModule.getCurrentState()
    };
  }

  /** Set module states to desired states; closed loop */
  private void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWERVEMODULECONSTANTS.MAX_SPEED);

    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** Set module states to desired states; open loop */
  private void setDesiredModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWERVEMODULECONSTANTS.MAX_SPEED);

    frontLeftModule.setDesiredStateOpenLoop(desiredStates[0]);
    frontRightModule.setDesiredStateOpenLoop(desiredStates[1]);
    backLeftModule.setDesiredStateOpenLoop(desiredStates[2]);
    backRightModule.setDesiredStateOpenLoop(desiredStates[3]);
  }

  /** */
  public void setDesiredSpeeds(ChassisSpeeds speeds) {
    setDesiredModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  /** */
  public void setDesiredSpeedsOpenLoop(ChassisSpeeds speeds) {
    setDesiredModuleStatesOpenLoop(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * 
   * @param vx meters per second
   * @param vy meters per second
   * @param omega degrees per second
   */
  public void driveRobotCentric(
    double vx,
    double vy,
    double omega,
    boolean openLoopDrive
  ) {
    if (openLoopDrive) {
      setDesiredSpeedsOpenLoop(new ChassisSpeeds(vx, vy, omega));
    } else {
      setDesiredSpeeds(new ChassisSpeeds(vx, vy, omega));
    }
  }

  /**
   * 
   * @param vx
   * @param vy
   * @param omega
   */
  public void driveFieldCentric(
    double vx,
    double vy,
    double omega,
    boolean openLoopDrive
  ) {
    if (openLoopDrive) {
      setDesiredSpeedsOpenLoop(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawAsRotation2d()));
    } else {
      setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, gyro.getYawAsRotation2d()));
    }
  }

  /** */
  public void setIdleModuleStates() {
    setDesiredModuleStatesOpenLoop(DRIVETRAINCONSTANTS.IDLE_MODULE_CONFIGURATION);
  }

  /** */
  public void haltAllModules() {
    frontLeftModule.halt();
    frontRightModule.halt();
    backLeftModule.halt();
    backRightModule.halt();
  }

  public void updateOdometryPose(Pose2d updatedPose) {
    odometry.resetPosition(updatedPose, gyro.getYawAsRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(
      gyro.getYawAsRotation2d(),
      frontLeftModule.getCurrentState(),
      frontRightModule.getCurrentState(),
      backLeftModule.getCurrentState(),
      backRightModule.getCurrentState()
    );
  }
}
