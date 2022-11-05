package frc.robot;

import frc.robot.subsystems.DriveTrain;

public class Superstructure {
  private final DriveTrain driveTrain;

  public Superstructure(
    DriveTrain driveTrain
  ) {
    this.driveTrain = driveTrain;
  }

  public void haltEverything() {
    driveTrain.haltAllModules();
    // ...
  }
}
