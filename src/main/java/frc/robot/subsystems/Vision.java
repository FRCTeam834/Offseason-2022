// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VISIONCONSTANTS;

public class Vision extends SubsystemBase {
  // String name of network table
  private final String cameraName;
  private final PhotonCamera camera;
  /** Creates a new Vision. */
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
  public Pose2d getPoseFromVision() {
    // temporary
    Map<Integer, Transform3d> lookup = new HashMap<>();

    if(!camera.getLatestResult().hasTargets()) return null;
    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
    int id = target.getFiducialId();

    Transform3d fiducialTransform = lookup.get(id);
    Transform3d cameraToTarget = target.getBestCameraToTarget().inverse();
    Transform3d cameraToRobot = VISIONCONSTANTS.CAMERA_POS;

    Transform3d robotTransform = fiducialTransform.plus(cameraToTarget).plus(cameraToRobot);

    return new Pose2d(
      robotTransform.getTranslation().toTranslation2d(),
      robotTransform.getRotation().toRotation2d()
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
