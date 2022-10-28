package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.subsystems.SwerveModule;

/**
 * Not really a factory class but oh well
 */
public class SwerveModuleFactory {
  public static final SwerveModule getFLModule() {
    SwerveModule module = new SwerveModule(
      CANIDS.FL_STEER_ID,
      CANIDS.FL_DRIVE_ID,
      CANIDS.FL_CANCODER_ID
    );

    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 20)
      .configPositionControlP(PIDGAINS.FL_STEER_kP)
      .configPositionControlI(PIDGAINS.FL_STEER_kI)
      .configPositionControlD(PIDGAINS.FL_STEER_kD)
      .configPositionControlFF(PIDGAINS.FL_STEER_kFF);

    module.getDriveController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 10)
      .configPositionControlP(PIDGAINS.FL_DRIVE_kP)
      .configPositionControlI(PIDGAINS.FL_DRIVE_kI)
      .configPositionControlD(PIDGAINS.FL_DRIVE_kD)
      .configPositionControlFF(PIDGAINS.FL_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getFRModule() {
    SwerveModule module = new SwerveModule(
      CANIDS.FR_STEER_ID,
      CANIDS.FR_DRIVE_ID,
      CANIDS.FR_CANCODER_ID
    );

    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 20)
      .configPositionControlP(PIDGAINS.FR_STEER_kP)
      .configPositionControlI(PIDGAINS.FR_STEER_kI)
      .configPositionControlD(PIDGAINS.FR_STEER_kD)
      .configPositionControlFF(PIDGAINS.FR_STEER_kFF);

    module.getDriveController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 10)
      .configPositionControlP(PIDGAINS.FR_DRIVE_kP)
      .configPositionControlI(PIDGAINS.FR_DRIVE_kI)
      .configPositionControlD(PIDGAINS.FR_DRIVE_kD)
      .configPositionControlFF(PIDGAINS.FR_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBLModule() {
    SwerveModule module = new SwerveModule(
      CANIDS.BL_STEER_ID,
      CANIDS.BL_DRIVE_ID,
      CANIDS.BL_CANCODER_ID
    );

    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 20)
      .configPositionControlP(PIDGAINS.BL_STEER_kP)
      .configPositionControlI(PIDGAINS.BL_STEER_kI)
      .configPositionControlD(PIDGAINS.BL_STEER_kD)
      .configPositionControlFF(PIDGAINS.BL_STEER_kFF);

    module.getDriveController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 10)
      .configPositionControlP(PIDGAINS.BL_DRIVE_kP)
      .configPositionControlI(PIDGAINS.BL_DRIVE_kI)
      .configPositionControlD(PIDGAINS.BL_DRIVE_kD)
      .configPositionControlFF(PIDGAINS.BL_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBRModule() {
    SwerveModule module = new SwerveModule(
      CANIDS.BR_STEER_ID,
      CANIDS.BR_DRIVE_ID,
      CANIDS.BR_CANCODER_ID
    );

    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 20)
      .configPositionControlP(PIDGAINS.BR_STEER_kP)
      .configPositionControlI(PIDGAINS.BR_STEER_kI)
      .configPositionControlD(PIDGAINS.BR_STEER_kD)
      .configPositionControlFF(PIDGAINS.BR_STEER_kFF);

    module.getDriveController()
      .configRestoreFactoryDefaults()
      .configIdleMode(CANSparkMax.IdleMode.kBrake)
      .configInverted(false)
      .configVoltageCompensation(12)
      .configSmartCurrentLimit(20)
      .configPeriodicFramePeriods(255, 255, 10)
      .configPositionControlP(PIDGAINS.BR_DRIVE_kP)
      .configPositionControlI(PIDGAINS.BR_DRIVE_kI)
      .configPositionControlD(PIDGAINS.BR_DRIVE_kD)
      .configPositionControlFF(PIDGAINS.BR_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }
}
