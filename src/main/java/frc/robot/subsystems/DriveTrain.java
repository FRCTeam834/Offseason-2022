// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DRIVECONSTANTS;
import frc.robot.Constants.DRIVETRAINCONSTANTS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.utilities.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeftModule = SwerveModuleFactory.getFLModule();
    frontRightModule = SwerveModuleFactory.getFRModule();
    backLeftModule = SwerveModuleFactory.getBLModule();
    backRightModule = SwerveModuleFactory.getBRModule();

    SmartDashboard.putData("FLM", frontLeftModule);
    SmartDashboard.putData("FRM", frontRightModule);
    SmartDashboard.putData("BLM", backLeftModule);
    SmartDashboard.putData("BRM", backRightModule);

    kinematics = new SwerveDriveKinematics(
      DRIVETRAINCONSTANTS.FLM_POS,
      DRIVETRAINCONSTANTS.FRM_POS,
      DRIVETRAINCONSTANTS.BLM_POS,
      DRIVETRAINCONSTANTS.BRM_POS
    );
  }

  /** */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /** */
  public SwerveModule getFrontLeftModule() {
    return frontLeftModule;
  }

  /** */
  public SwerveModule getFrontRightModule() {
    return frontRightModule;
  }

  /** */
  public SwerveModule getBackLeftModule() {
    return backLeftModule;
  }

  /** */
  public SwerveModule getBackRightModule() {
    return backRightModule;
  }

  /** Set module states to desired states; closed loop */
  public void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** Set module states to desired states; open loop */
  public void setDesiredModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredStateOpenLoop(desiredStates[0]);
    frontRightModule.setDesiredStateOpenLoop(desiredStates[1]);
    backLeftModule.setDesiredStateOpenLoop(desiredStates[2]);
    backRightModule.setDesiredStateOpenLoop(desiredStates[3]);
  }

  /** */
  public void setDesiredSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
    DriveTrain.desaturateWheelSpeeds(desiredStates, speeds);
    setDesiredModuleStates(desiredStates);
  }

  /** */
  public void setDesiredSpeedsOpenLoop(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(speeds);
    DriveTrain.desaturateWheelSpeeds(desiredStates, speeds);
    setDesiredModuleStatesOpenLoop(kinematics.toSwerveModuleStates(speeds));
  }

  /**
   * 
   * @param vx meters per second
   * @param vy meters per second
   * @param omega degrees per second
   */
  public void driveRobotCentricOpenLoop(
    double vx,
    double vy,
    double omega
  ) {
    setDesiredSpeedsOpenLoop(new ChassisSpeeds(vx, vy, omega));
  }

  /**
   * 
   * @param vx meters per second
   * @param vy meters per second
   * @param omega degrees per second
   */
  public void driveRobotCentricClosedLoop(
    double vx,
    double vy,
    double omega
  ) {
    setDesiredSpeeds(new ChassisSpeeds(vx, vy, omega));
  }

  /**
   * 
   * @param vx
   * @param vy
   * @param omega
   */
  public void driveFieldCentricOpenLoop(
    double vx,
    double vy,
    double omega,
    DoubleSupplier robotYaw
  ) {
    setDesiredSpeedsOpenLoop(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(robotYaw.getAsDouble())));
  }

  /**
   * 
   * @param vx
   * @param vy
   * @param omega
   */
  public void driveFieldCentricClosedLoop(
    double vx,
    double vy,
    double omega,
    DoubleSupplier robotYaw
  ) {
    setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(robotYaw.getAsDouble())));
  }

  /** */
  public void setIdleModuleStates() {
    if (DRIVETRAINCONSTANTS.IDLE_MODULE_CONFIGURATION == null) {
      haltAllModules();
      return;
    }
    setDesiredModuleStatesOpenLoop(DRIVETRAINCONSTANTS.IDLE_MODULE_CONFIGURATION);
  }

  /** */
  public void haltAllModules() {
    frontLeftModule.halt();
    frontRightModule.halt();
    backLeftModule.halt();
    backRightModule.halt();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Better desaturateWheelSpeeds
   * Credit to 2363
   * https://www.chiefdelphi.com/t/good-vs-bad-swerve/414439/32
   * @param moduleStates
   */
  public static final void desaturateWheelSpeeds(SwerveModuleState[] moduleStates, ChassisSpeeds speeds) {
    double translationalK = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / DRIVECONSTANTS.MAX_TRANSLATIONAL_SPEED;
    double rotationalK = Math.abs(speeds.omegaRadiansPerSecond) / DRIVECONSTANTS.MAX_STEER_SPEED;
    double k = Math.max(translationalK, rotationalK);
  
    // Find the how fast the fastest spinning drive motor is spinning                                       
    double realMaxSpeed = 0.0;
    for (SwerveModuleState moduleState : moduleStates) {
      realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
    }
  
    double scale = Math.min(k * SWERVEMODULECONSTANTS.MAX_SPEED / realMaxSpeed, 1);
    for (SwerveModuleState moduleState : moduleStates) {
      moduleState.speedMetersPerSecond *= scale;
    }
  }
}
