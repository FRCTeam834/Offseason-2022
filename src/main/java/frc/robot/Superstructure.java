package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;

public class Superstructure extends SubsystemBase {
  
  public final DriveTrain driveTrain;
  public final Pigeon gyro;
  public final Vision vision;

  private SwerveDrivePoseEstimator poseEstimator;

  public Superstructure(
    DriveTrain driveTrain,
    Pigeon gyro,
    Vision vision
  ) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.vision = vision;

    poseEstimator = new SwerveDrivePoseEstimator(
      new Rotation2d(),
      new Pose2d(),
      driveTrain.getKinematics(),
      Constants.STATE_STDDEVS,
      Constants.LOCAL_STDDEVS,
      Constants.VISION_STDDEVS,
      0.02 // 20ms per loop
    );
  }

  /** */
  public Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Updates current robot pose using vision, and odometry as fallback
   */
  public void updateRobotPose() {
    poseEstimator.update(
      gyro.getYawAsRotation2d(),
      driveTrain.getFrontLeftModule().getCurrentState(),
      driveTrain.getFrontRightModule().getCurrentState(),
      driveTrain.getBackLeftModule().getCurrentState(),
      driveTrain.getBackRightModule().getCurrentState()
    );

    poseEstimator.addVisionMeasurement(vision.getPose2dFromVision(), Timer.getFPGATimestamp() / 1e-6);
  }

  /** */
  public void resetRobotPose(Pose2d newPose) {
    poseEstimator.resetPosition(newPose, gyro.getYawAsRotation2d());
  }

  /** */
  public void haltEverything() {
    driveTrain.haltAllModules();
    // ...
  }

  @Override
  public void periodic() {
    updateRobotPose();
  }
}
