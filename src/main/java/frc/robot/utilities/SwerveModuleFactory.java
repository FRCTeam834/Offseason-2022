package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.subsystems.SwerveModule;

/**
 * "Factory" class
 */
public class SwerveModuleFactory {
  /** Controller defaults */
  public static final class STEERDEFAULTS {
    public static final CANSparkMax.IdleMode IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double NOMINALVOLTAGE = 12;
    public static final int SMARTCURRENTLIMIT = 20;
    public static final int driveSmartCurrentLimit = 20;

    public static final int CONTROLFRAMEPERIOD = 10;
    public static final int PERIODICFRAMEk0 = 65535;
    public static final int PERIODICFRAMEk1 = 65535;
    public static final int PERIODICFRAMEk2 = 20;
    public static final int PERIODICFRAMEk3 = 65535;
  }
  public static final class DRIVEDEFAULTS {
    public static final CANSparkMax.IdleMode IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double NOMINALVOLTAGE = 12;
    public static final int SMARTCURRENTLIMIT = 20;
    public static final int driveSmartCurrentLimit = 20;

    public static final int CONTROLFRAMEPERIOD = 10;
    public static final int PERIODICFRAMEk0 = 65535;
    public static final int PERIODICFRAMEk1 = 65535;
    public static final int PERIODICFRAMEk2 = 20;
    public static final int PERIODICFRAMEk3 = 65535;
  }
  
  private static final SwerveModule buildDefaults(SwerveModule module) {
    module.getSteerController()
      .configRestoreFactoryDefaults()
      .configIdleMode(STEERDEFAULTS.IDLEMODE)
      .configInverted(false)
      .configVoltageCompensation(STEERDEFAULTS.NOMINALVOLTAGE)
      .configSmartCurrentLimit(STEERDEFAULTS.SMARTCURRENTLIMIT)
      .configPositionConversionFactor(SWERVEMODULECONSTANTS.POSITION_CONVERSION_FACTOR)
      .configControlFramePeriod(STEERDEFAULTS.CONTROLFRAMEPERIOD)
      .configPeriodicFramePeriods(
        STEERDEFAULTS.PERIODICFRAMEk0,
        STEERDEFAULTS.PERIODICFRAMEk1,
        STEERDEFAULTS.PERIODICFRAMEk2,
        STEERDEFAULTS.PERIODICFRAMEk3);

    module.getDriveController()
    .configRestoreFactoryDefaults()
    .configIdleMode(DRIVEDEFAULTS.IDLEMODE)
    .configInverted(false)
    .configVoltageCompensation(DRIVEDEFAULTS.NOMINALVOLTAGE)
    .configSmartCurrentLimit(DRIVEDEFAULTS.SMARTCURRENTLIMIT)
    .configPositionConversionFactor(SWERVEMODULECONSTANTS.VELOCITY_CONVERSION_FACTOR)
    .configControlFramePeriod(DRIVEDEFAULTS.CONTROLFRAMEPERIOD)
    .configPeriodicFramePeriods(
      DRIVEDEFAULTS.PERIODICFRAMEk0,
      DRIVEDEFAULTS.PERIODICFRAMEk1,
      DRIVEDEFAULTS.PERIODICFRAMEk2,
      DRIVEDEFAULTS.PERIODICFRAMEk3);

    return module;
  }

  public static final SwerveModule getFLModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      "FrontLeft",
      CANIDS.FL_STEER_ID,
      CANIDS.FL_DRIVE_ID,
      CANIDS.FL_CANCODER_ID,
      SWERVEMODULECONSTANTS.FL_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.FL_STEER.kP)
      .configPositionControlI(PIDGAINS.FL_STEER.kI)
      .configPositionControlD(PIDGAINS.FL_STEER.kD)
      .configPositionControlFF(PIDGAINS.FL_STEER.kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FL_DRIVE.kP)
      .configVelocityControlI(PIDGAINS.FL_DRIVE.kI)
      .configVelocityControlD(PIDGAINS.FL_DRIVE.kD)
      .configVelocityControlFF(PIDGAINS.FL_DRIVE.kFF);

    if (DriverStation.isFMSAttached() || Constants.BURNFLASHES) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getFRModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      "FrontRight",
      CANIDS.FR_STEER_ID,
      CANIDS.FR_DRIVE_ID,
      CANIDS.FR_CANCODER_ID,
      SWERVEMODULECONSTANTS.FR_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.FR_STEER.kP)
      .configPositionControlI(PIDGAINS.FR_STEER.kI)
      .configPositionControlD(PIDGAINS.FR_STEER.kD)
      .configPositionControlFF(PIDGAINS.FR_STEER.kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FR_DRIVE.kP)
      .configVelocityControlI(PIDGAINS.FR_DRIVE.kI)
      .configVelocityControlD(PIDGAINS.FR_DRIVE.kD)
      .configVelocityControlFF(PIDGAINS.FR_DRIVE.kFF);

    if (DriverStation.isFMSAttached() || Constants.BURNFLASHES) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBLModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      "BackLeft",
      CANIDS.BL_STEER_ID,
      CANIDS.BL_DRIVE_ID,
      CANIDS.BL_CANCODER_ID,
      SWERVEMODULECONSTANTS.BL_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.BL_STEER.kP)
      .configPositionControlI(PIDGAINS.BL_STEER.kI)
      .configPositionControlD(PIDGAINS.BL_STEER.kD)
      .configPositionControlFF(PIDGAINS.BL_STEER.kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BL_DRIVE.kP)
      .configVelocityControlI(PIDGAINS.BL_DRIVE.kI)
      .configVelocityControlD(PIDGAINS.BL_DRIVE.kD)
      .configVelocityControlFF(PIDGAINS.BL_DRIVE.kFF);

    if (DriverStation.isFMSAttached() || Constants.BURNFLASHES) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }

  public static final SwerveModule getBRModule() {
    SwerveModule module = SwerveModuleFactory.buildDefaults(new SwerveModule(
      "BackRight",
      CANIDS.BR_STEER_ID,
      CANIDS.BR_DRIVE_ID,
      CANIDS.BR_CANCODER_ID,
      SWERVEMODULECONSTANTS.BR_MAGNET_OFFSET
    ));

    module.getSteerController()
      .configPositionControlP(PIDGAINS.BR_STEER.kP)
      .configPositionControlI(PIDGAINS.BR_STEER.kI)
      .configPositionControlD(PIDGAINS.BR_STEER.kD)
      .configPositionControlFF(PIDGAINS.BR_STEER.kFF);

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BR_DRIVE.kP)
      .configVelocityControlI(PIDGAINS.BR_DRIVE.kI)
      .configVelocityControlD(PIDGAINS.BR_DRIVE.kD)
      .configVelocityControlFF(PIDGAINS.BR_DRIVE.kFF);

    if (DriverStation.isFMSAttached() || Constants.BURNFLASHES) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }
}
