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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  // String name of network table
  private final PhotonCamera camera;
  /** Creates a new Vision. */
  public Vision(String name) {
    camera = new PhotonCamera(name);
  }

  public Pose2d getPoseFromVision() {
    // temporary
    Map<Integer, Transform3d> lookup = new HashMap<>();

    if(!camera.getLatestResult().hasTargets()) return null;
    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
    int id = target.getFiducialId();

    // Currently assumes camera at center of robot
    Transform3d transform = lookup.get(id).plus(target.getBestCameraToTarget().inverse());

    return new Pose2d(
      transform.getTranslation().toTranslation2d(),
      transform.getRotation().toRotation2d()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
