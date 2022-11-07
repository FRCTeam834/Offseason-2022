// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VISIONCONSTANTS;

public class Vision extends SubsystemBase {

  private final String cameraName;
  private final PhotonCamera camera;
  /**
   * 
   * @param cameraName String name of network table
   */
  public Vision(String cameraName) {
    this.cameraName = cameraName;
    camera = new PhotonCamera(this.cameraName);
  }

  /** */
  public boolean hasTarget() {
    return camera.getLatestResult().hasTargets();
  }

  /**
   * 
   * Get robot position based on vision results
   * @return
   */
  public Pose2d getPose2dFromVision() {
    return getPoseFromVision().toPose2d();
  }

  /**
   * 
   * @return
   */
  public Pose3d getPoseFromVision() {
    // temporary; will be provided by wpilib later
    Map<Integer, Pose3d> AprilTagLookup = new HashMap<>();

    // Currently Simple filtering strategy
    // Removes poses with high pose ambiguity
    PhotonPipelineResult latestResult = camera.getLatestResult();
    if(!latestResult.hasTargets()) return null;

    List<PhotonTrackedTarget> allTargets = latestResult.getTargets();

    PhotonTrackedTarget bestTarget = null;

    for (PhotonTrackedTarget target : allTargets) {
      if (target.getPoseAmbiguity() > 0.2) continue;

      bestTarget = target;
      break;
    }

    if (bestTarget == null) return null;

    // Get Pose3d from target
    int tagID = bestTarget.getFiducialId();

    Pose3d tagPose = AprilTagLookup.get(tagID);
    Transform3d fiducialTransform = new Transform3d(tagPose.getTranslation(), tagPose.getRotation());;
    Transform3d targetToCamera = bestTarget.getBestCameraToTarget().inverse(); // Take inverse to find "targetToCamera"
    Transform3d cameraToRobot = VISIONCONSTANTS.CAMERA_POS;

    Transform3d robotTransform = fiducialTransform.plus(targetToCamera).plus(cameraToRobot);

    return new Pose3d(
      robotTransform.getTranslation(),
      robotTransform.getRotation()
    );
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    if (Constants.telemetry == false) return;

    builder.setSmartDashboardType("Vision" + cameraName);
    builder.addBooleanProperty("hasTarget", this::hasTarget, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
