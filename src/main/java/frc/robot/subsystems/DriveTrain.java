// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.PIDS.SwerveModulePIDS;

public class DriveTrain extends SubsystemBase {
  private static final DriveTrain instance = new DriveTrain();

  private static final DriveTrain getInstance() {
    return instance;
  }

  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontLeftModule = new SwerveModule(
      CANIDS.FL_STEER_ID,
      CANIDS.FL_DRIVE_ID,
      CANIDS.FL_CANCODER_ID
    );
    frontRightModule = new SwerveModule(
      CANIDS.FR_STEER_ID,
      CANIDS.FR_DRIVE_ID,
      CANIDS.FR_CANCODER_ID
    );
    backLeftModule = new SwerveModule(
      CANIDS.BL_STEER_ID,
      CANIDS.BL_DRIVE_ID,
      CANIDS.BL_CANCODER_ID
    );
    backRightModule = new SwerveModule(
      CANIDS.BR_STEER_ID,
      CANIDS.BR_DRIVE_ID,
      CANIDS.BR_CANCODER_ID
    );

    frontLeftModule.setSteerP(SwerveModulePIDS.FL_STEER_kP);
    frontLeftModule.setSteerI(SwerveModulePIDS.FL_STEER_kI);
    frontLeftModule.setSteerD(SwerveModulePIDS.FL_STEER_kD);

    frontRightModule.setSteerP(SwerveModulePIDS.FR_STEER_kP);
    frontRightModule.setSteerI(SwerveModulePIDS.FR_STEER_kI);
    frontRightModule.setSteerD(SwerveModulePIDS.FR_STEER_kD);

    backLeftModule.setSteerP(SwerveModulePIDS.BL_STEER_kP);
    backLeftModule.setSteerI(SwerveModulePIDS.BL_STEER_kI);
    backLeftModule.setSteerD(SwerveModulePIDS.BL_STEER_kD);

    backRightModule.setSteerP(SwerveModulePIDS.FL_STEER_kP);
    backRightModule.setSteerI(SwerveModulePIDS.BR_STEER_kI);
    backRightModule.setSteerD(SwerveModulePIDS.BR_STEER_kD);

    if (DriverStation.isFMSAttached()) {
      frontLeftModule.getSteerController().burnFlash();
      frontLeftModule.getDriveController().burnFlash();

      frontRightModule.getSteerController().burnFlash();
      frontRightModule.getDriveController().burnFlash();

      backLeftModule.getSteerController().burnFlash();
      backLeftModule.getDriveController().burnFlash();

      backRightModule.getSteerController().burnFlash();
      backRightModule.getDriveController().burnFlash();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
