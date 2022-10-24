// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.libs.CANMotorController;

public class SwerveModule extends SubsystemBase {
  /** */
  public static final class SwerveModuleConfig {
    public int steerID, driveID, CANCoderID;
    /** */
    public SwerveModuleConfig(
      int steerID,
      int driveID,
      int CANCoderID
    ) {
      this.steerID = steerID;
      this.driveID = driveID;
      this.CANCoderID = CANCoderID;
    }
  }

  private final CANMotorController steerController;
  private final CANMotorController driveController;
  /** Creates a new SwerveModule. */
  public SwerveModule(SwerveModuleConfig config) {
    steerController = new CANMotorController(config.steerID);
    driveController = new CANMotorController(config.driveID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
