package frc.robot.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose3d;

public class AprilTagsLookup {
  // HashMap containing 3d location data of each apriltag
  private final Map<Integer, Pose3d> map;

  public AprilTagsLookup() {
    map = new HashMap<>();
    // !TODO: Parse from .json file?
  }

  public Pose3d getTagPose3d(int tagID) {
    return map.get(tagID);
  }
}
