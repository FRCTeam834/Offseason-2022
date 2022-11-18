package frc.robot;

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

    Pose2d poseFromVision = vision.getPose2dFromVision();
    if (poseFromVision != null) {
      poseEstimator.addVisionMeasurement(
        vision.getPose2dFromVision(),
        (Timer.getFPGATimestamp() / 1e-6) - (vision.getCameraLatencyMs() / 1000.0)
      );
    }
  }

  /** */
  public void resetRobotPose(Pose2d newPose) {
    poseEstimator.resetPosition(newPose, gyro.getYawAsRotation2d());
  }

  /** Should only be used once at start of auton */
  public void resetRobotPoseAndGyro(Pose2d newPose) {
    gyro.setYaw(newPose.getRotation().getDegrees());
    poseEstimator.resetPosition(newPose, newPose.getRotation());
  }

  /** */
  public void haltEverything() {
    driveTrain.haltAllModules();
    // ...
  }

  @Override
  public void periodic() {
    // updateRobotPose();
  }
}
