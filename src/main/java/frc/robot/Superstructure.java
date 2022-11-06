package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;

public class Superstructure extends SubsystemBase {
  
  public final DriveTrain driveTrain;
  public final Pigeon gyro;
  public final Vision vision;

  private Pose2d robotPose;

  public Superstructure(
    DriveTrain driveTrain,
    Pigeon gyro,
    Vision vision
  ) {
    this.driveTrain = driveTrain;
    this.gyro = gyro;
    this.vision = vision;
  }

  public void resetOdometryPose(Pose2d newPose) {
    gyro.setYaw(newPose.getRotation().getDegrees());
    driveTrain.resetOdometry(newPose);
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public void updateRobotPose() {
    driveTrain.updateOdometry(gyro.getYaw());
    Pose2d poseFromVision = vision.getPoseFromVision();
    Pose2d poseFromOdometry = driveTrain.getPoseFromOdometry();
    
    if (poseFromVision == null) {
      robotPose = poseFromOdometry;
    } else {
      resetOdometryPose(poseFromVision);
      robotPose = poseFromVision;
    }
  }

  public void haltEverything() {
    driveTrain.haltAllModules();
    // ...
  }

  @Override
  public void periodic() {
    updateRobotPose();
  }
}
