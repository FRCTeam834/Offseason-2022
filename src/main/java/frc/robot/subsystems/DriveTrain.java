// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVETRAINCONSTANTS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.utilities.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
  private static final DriveTrain instance = new DriveTrain();
  /** */
  public static final DriveTrain getInstance() {
    return instance;
  }

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics;

  /** Creates a new DriveTrain. */
  private DriveTrain() {
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
  }

  /** Set module states to desired states; closed loop */
  private void setDesiredModuleStates(SwerveModuleState[] desiredStates, boolean openLoopDrive) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SWERVEMODULECONSTANTS.MAX_SPEED);

    frontLeftModule.setDesiredState(desiredStates[0], openLoopDrive);
    frontRightModule.setDesiredState(desiredStates[1], openLoopDrive);
    backLeftModule.setDesiredState(desiredStates[2], openLoopDrive);
    backRightModule.setDesiredState(desiredStates[3], openLoopDrive);
  }

  /** */
  public void setDesiredSpeeds(ChassisSpeeds speeds, boolean openLoopDrive) {
    setDesiredModuleStates(kinematics.toSwerveModuleStates(speeds), openLoopDrive);
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
    double omega
  ) {
    setDesiredSpeeds(new ChassisSpeeds(vx, vy, omega), false);
  }

  /** */
  public void setIdleModuleStates() {
    setDesiredModuleStates(DRIVETRAINCONSTANTS.IDLE_MODULE_CONFIGURATION, false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
