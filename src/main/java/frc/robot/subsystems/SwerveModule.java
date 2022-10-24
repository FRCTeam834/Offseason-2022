// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.libs.CANMotorController;

public class SwerveModule extends SubsystemBase {

  private final CANMotorController steerController;
  private final CANMotorController driveController;
  /** Creates a new SwerveModule. */
  public SwerveModule(
    int steerID,
    int driveID,
    int CANCoderID
  ) {
    steerController = new CANMotorController(steerID);
    driveController = new CANMotorController(driveID);
    
    // Config motor controllers
    steerController.configIdleMode(CANSparkMax.IdleMode.kBrake);
    steerController.configVoltageCompensation(SwerveModuleConstants.NOMINAL_VOLTAGE);
    steerController.configSmartCurrentLimit(SwerveModuleConstants.CURRENT_LIMIT);
    steerController.configPeriodicFramePeriods(
      SwerveModuleConstants.PERIODIC_FRAME_k0,
      SwerveModuleConstants.PERIODIC_FRAME_k1,
      SwerveModuleConstants.PERIODIC_FRAME_k2
    );
  }

  /** */
  public CANMotorController getSteerController() {
    return steerController;
  }

  /** */
  public CANMotorController getDriveController() {
    return driveController;
  }

  /** */
  public void setSteerP(double kP) {
    steerController.setP(kP);
  }
  /** */
  public void setSteerI(double kI) {
    steerController.setI(kI);
  }
  /** */
  public void setSteerD(double kD) {
    steerController.setD(kD);
  }
  /** */
  public void setDriveP(double kP) {
    driveController.setP(kP);
  }
  /** */
  public void setDriveI(double kI) {
    driveController.setI(kI);
  }
  /** */
  public void setDriveD(double kD) {
    driveController.setP(kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    steerController.periodic(
      SwerveModuleConstants.AT_VELOCITY_TOLERANCE,
      SwerveModuleConstants.AT_POSITION_TOLERANCE
    );
    driveController.periodic(
      SwerveModuleConstants.AT_VELOCITY_TOLERANCE,
      SwerveModuleConstants.AT_POSITION_TOLERANCE
    );
  }
}
