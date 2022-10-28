// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.SparkMaxController;

public class SwerveModule extends SubsystemBase {
  private final SparkMaxController steerController;
  private final SparkMaxController driveController;
  /** Creates a new SwerveModule. */
  public SwerveModule(
    int steerCANID,
    int driveCANID,
    int CANCoderID
  ) {
    steerController = new SparkMaxController(steerCANID, 0, 0);
    driveController = new SparkMaxController(steerCANID, 10, 5);
  }
  
  public SparkMaxController getSteerController() {
    return steerController;
  }

  public SparkMaxController getDriveController() {
    return driveController;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
