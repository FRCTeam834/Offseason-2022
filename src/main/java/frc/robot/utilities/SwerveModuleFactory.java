package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.subsystems.SwerveModule;

/**
 * 
 * Not really a factory class but oh well
 * 
 */
public class SwerveModuleFactory {

  private static final CANSparkMax.IdleMode idleMode = CANSparkMax.IdleMode.kBrake;
  private static final boolean isInverted = false;
  private static final double nominalVoltage = 12;
  private static final int smartCurrentLimit = 20;

  private static final int steerPeriodicFramek0 = 255;
  private static final int steerPeriodicFramek1 = 255;
  private static final int steerPeriodicFramek2 = 20;

  private static final int drivePeriodicFramek0 = 255;
  private static final int drivePeriodicFramek1 = 255;
  private static final int drivePeriodicFramek2 = 10;

  private static final SwerveModule buildDefaults(SwerveModule module) {
    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(idleMode)
      .configInverted(isInverted)
      .configVoltageCompensation(nominalVoltage)
      .configSmartCurrentLimit(smartCurrentLimit)
      .configPositionConversionFactor(SWERVEMODULECONSTANTS.POSITION_CONVERSION_FACTOR)
      .configPeriodicFramePeriods(steerPeriodicFramek0, steerPeriodicFramek1, steerPeriodicFramek2);

    module.getDriveController()
      .configRestoreFactoryDefaults()
      .configIdleMode(idleMode)
      .configInverted(isInverted)
      .configVoltageCompensation(nominalVoltage)
      .configSmartCurrentLimit(smartCurrentLimit)
      .configVelocityConversionFactor(SWERVEMODULECONSTANTS.VELOCITY_CONVERSION_FACTOR)
      .configPeriodicFramePeriods(drivePeriodicFramek0, drivePeriodicFramek1, drivePeriodicFramek2);

    return module;
  }

  public static final SwerveModule getFLModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      CANIDS.FL_STEER_ID,
      CANIDS.FL_DRIVE_ID,
      CANIDS.FL_CANCODER_ID,
      SWERVEMODULECONSTANTS.FL_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.FL_STEER_kP)
      .configPositionControlI(PIDGAINS.FL_STEER_kI)
      .configPositionControlD(PIDGAINS.FL_STEER_kD)
      .configPositionControlFF(PIDGAINS.FL_STEER_kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FL_DRIVE_kP)
      .configVelocityControlI(PIDGAINS.FL_DRIVE_kI)
      .configVelocityControlD(PIDGAINS.FL_DRIVE_kD)
      .configVelocityControlFF(PIDGAINS.FL_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getFRModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      CANIDS.FR_STEER_ID,
      CANIDS.FR_DRIVE_ID,
      CANIDS.FR_CANCODER_ID,
      SWERVEMODULECONSTANTS.FR_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.FR_STEER_kP)
      .configPositionControlI(PIDGAINS.FR_STEER_kI)
      .configPositionControlD(PIDGAINS.FR_STEER_kD)
      .configPositionControlFF(PIDGAINS.FR_STEER_kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FR_DRIVE_kP)
      .configVelocityControlI(PIDGAINS.FR_DRIVE_kI)
      .configVelocityControlD(PIDGAINS.FR_DRIVE_kD)
      .configVelocityControlFF(PIDGAINS.FR_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBLModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      CANIDS.BL_STEER_ID,
      CANIDS.BL_DRIVE_ID,
      CANIDS.BL_CANCODER_ID,
      SWERVEMODULECONSTANTS.BL_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.BL_STEER_kP)
      .configPositionControlI(PIDGAINS.BL_STEER_kI)
      .configPositionControlD(PIDGAINS.BL_STEER_kD)
      .configPositionControlFF(PIDGAINS.BL_STEER_kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BL_DRIVE_kP)
      .configVelocityControlI(PIDGAINS.BL_DRIVE_kI)
      .configVelocityControlD(PIDGAINS.BL_DRIVE_kD)
      .configVelocityControlFF(PIDGAINS.BL_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBRModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      CANIDS.BR_STEER_ID,
      CANIDS.BR_DRIVE_ID,
      CANIDS.BR_CANCODER_ID,
      SWERVEMODULECONSTANTS.BR_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.BR_STEER_kP)
      .configPositionControlI(PIDGAINS.BR_STEER_kI)
      .configPositionControlD(PIDGAINS.BR_STEER_kD)
      .configPositionControlFF(PIDGAINS.BR_STEER_kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BR_DRIVE_kP)
      .configVelocityControlI(PIDGAINS.BR_DRIVE_kI)
      .configVelocityControlD(PIDGAINS.BR_DRIVE_kD)
      .configVelocityControlFF(PIDGAINS.BR_DRIVE_kFF);

    if (DriverStation.isFMSAttached()) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }
}
