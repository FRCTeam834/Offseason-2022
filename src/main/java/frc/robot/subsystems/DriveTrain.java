// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DRIVECONSTANTS;
import frc.robot.Constants.DRIVETRAINCONSTANTS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.utilities.SwerveModuleFactory;

public class DriveTrain extends SubsystemBase {
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

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeftModule = SwerveModuleFactory.getFLModule(false);
    frontRightModule = SwerveModuleFactory.getFRModule(false);
    backLeftModule = SwerveModuleFactory.getBLModule(false);
    backRightModule = SwerveModuleFactory.getBRModule(false);

    kinematics = new SwerveDriveKinematics(
      DRIVETRAINCONSTANTS.FLM_POS,
      DRIVETRAINCONSTANTS.FRM_POS,
      DRIVETRAINCONSTANTS.BLM_POS,
      DRIVETRAINCONSTANTS.BRM_POS
    );

    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d());
  }

  /** Set module states to desired states; closed loop */
  private void setDesiredModuleStates(SwerveModuleState[] desiredStates) {
    frontLeftModule.setDesiredState(desiredStates[0]);
    frontRightModule.setDesiredState(desiredStates[1]);
    backLeftModule.setDesiredState(desiredStates[2]);
    backRightModule.setDesiredState(desiredStates[3]);
  }

  /** Set module states to desired states; open loop */
  private void setDesiredModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
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
    DoubleSupplier robotYaw,
    boolean openLoopDrive
  ) {
    if (openLoopDrive) {
      setDesiredSpeedsOpenLoop(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(robotYaw.getAsDouble())));
    } else {
      setDesiredSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, Rotation2d.fromDegrees(robotYaw.getAsDouble())));
    }
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

  public Pose2d getPoseFromOdometry() {
    return odometry.getPoseMeters();
  }

  public void updateOdometry(double robotYaw) {
    odometry.update(
      Rotation2d.fromDegrees(robotYaw),
      frontLeftModule.getCurrentState(),
      frontRightModule.getCurrentState(),
      backLeftModule.getCurrentState(),
      backRightModule.getCurrentState()
    );
  }

  public void resetOdometry(Pose2d newPose) {
    odometry.resetPosition(newPose, newPose.getRotation());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetry == false) return;

    builder.setSmartDashboardType("DriveTrain");
    builder.addDoubleArrayProperty("FLM", frontLeftModule::telemetryGetState, null);
    builder.addDoubleArrayProperty("FRM", frontRightModule::telemetryGetState, null);
    builder.addDoubleArrayProperty("BLM", backLeftModule::telemetryGetState, null);
    builder.addDoubleArrayProperty("BRM", backRightModule::telemetryGetState, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * 
   * Path following command
   * @param trajectory path
   * @param resetOdometry reset odometry - set as true if this is the first path
   * @return
   */
  public Command getFollowPathCommand(
    PathPlannerTrajectory trajectory,
    Supplier<Pose2d> robotPoseSupplier,
    boolean resetOdometry
  ) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (resetOdometry) {
          resetOdometry(trajectory.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        robotPoseSupplier,
        kinematics,
        PIDGAINS.AUTON_X.generateController(),
        PIDGAINS.AUTON_Y.generateController(),
        PIDGAINS.AUTON_STEER.generateController(),
        this::setDesiredModuleStates,
        // subsystem requirements
        this)
    );
  }
}
