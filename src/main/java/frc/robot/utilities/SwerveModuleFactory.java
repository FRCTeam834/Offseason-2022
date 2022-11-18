package frc.robot.utilities;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.CANIDS;
import frc.robot.Constants.FFGAINS;
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
    public static final int PERIODICFRAMEk1 = 20;
    public static final int PERIODICFRAMEk2 = 20;
    public static final int PERIODICFRAMEk3 = 65535;
  }
  public static final class DRIVEDEFAULTS {
    public static final CANSparkMax.IdleMode IDLEMODE = CANSparkMax.IdleMode.kBrake;
    public static final double NOMINALVOLTAGE = 12;
    public static final int SMARTCURRENTLIMIT = 40;
    public static final int driveSmartCurrentLimit = 20;

    public static final int CONTROLFRAMEPERIOD = 10;
    public static final int PERIODICFRAMEk0 = 65535;
    public static final int PERIODICFRAMEk1 = 20;
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
      .configPositionControlP(PIDGAINS.FL_STEER.getP())
      .configPositionControlI(PIDGAINS.FL_STEER.getI())
      .configPositionControlD(PIDGAINS.FL_STEER.getD());

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FL_DRIVE.getP())
      .configVelocityControlI(PIDGAINS.FL_DRIVE.getI())
      .configVelocityControlD(PIDGAINS.FL_DRIVE.getD());

    module.setSteerFeedforward(FFGAINS.FL_STEER_FF);
    module.setDriveFeedforward(FFGAINS.FL_DRIVE_FF);
    

    if (Constants.tuningMode) {
      module.setSteerPIDSupplier(PIDGAINS.FL_STEER::getGains);
      module.setDrivePIDSupplier(PIDGAINS.FL_DRIVE::getGains);

      module.setSteerPIDChangedSupplier(PIDGAINS.FL_STEER::hasChanged);
      module.setDrivePIDChangedSupplier(PIDGAINS.FL_DRIVE::hasChanged);
    }

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
      .configPositionControlP(PIDGAINS.FR_STEER.getP())
      .configPositionControlI(PIDGAINS.FR_STEER.getI())
      .configPositionControlD(PIDGAINS.FR_STEER.getD());

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.FR_DRIVE.getP())
      .configVelocityControlI(PIDGAINS.FR_DRIVE.getI())
      .configVelocityControlD(PIDGAINS.FR_DRIVE.getD());

    module.setSteerFeedforward(FFGAINS.FR_STEER_FF);
    module.setDriveFeedforward(FFGAINS.FR_DRIVE_FF);

    if (Constants.tuningMode) {
      module.setSteerPIDSupplier(PIDGAINS.FR_STEER::getGains);
      module.setDrivePIDSupplier(PIDGAINS.FR_DRIVE::getGains);

      module.setSteerPIDChangedSupplier(PIDGAINS.FR_STEER::hasChanged);
      module.setDrivePIDChangedSupplier(PIDGAINS.FR_DRIVE::hasChanged);
    }

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
      .configPositionControlP(PIDGAINS.BL_STEER.getP())
      .configPositionControlI(PIDGAINS.BL_STEER.getI())
      .configPositionControlD(PIDGAINS.BL_STEER.getD());

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BL_DRIVE.getP())
      .configVelocityControlI(PIDGAINS.BL_DRIVE.getI())
      .configVelocityControlD(PIDGAINS.BL_DRIVE.getD());

    module.setSteerFeedforward(FFGAINS.BL_STEER_FF);
    module.setDriveFeedforward(FFGAINS.BL_DRIVE_FF);

    if (Constants.tuningMode) {
      module.setSteerPIDSupplier(PIDGAINS.BL_STEER::getGains);
      module.setDrivePIDSupplier(PIDGAINS.BL_DRIVE::getGains);

      module.setSteerPIDChangedSupplier(PIDGAINS.BL_STEER::hasChanged);
      module.setDrivePIDChangedSupplier(PIDGAINS.BL_DRIVE::hasChanged);
    }

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
      .configPositionControlP(PIDGAINS.BR_STEER.getP())
      .configPositionControlI(PIDGAINS.BR_STEER.getI())
      .configPositionControlD(PIDGAINS.BR_STEER.getD());

    module.getDriveController()
      .configVelocityControlP(PIDGAINS.BR_DRIVE.getP())
      .configVelocityControlI(PIDGAINS.BR_DRIVE.getI())
      .configVelocityControlD(PIDGAINS.BR_DRIVE.getD());

    module.setSteerFeedforward(FFGAINS.BR_STEER_FF);
    module.setDriveFeedforward(FFGAINS.BR_DRIVE_FF);

    if (Constants.tuningMode) {
      module.setSteerPIDSupplier(PIDGAINS.BR_STEER::getGains);
      module.setDrivePIDSupplier(PIDGAINS.BR_DRIVE::getGains);

      module.setSteerPIDChangedSupplier(PIDGAINS.BR_STEER::hasChanged);
      module.setDrivePIDChangedSupplier(PIDGAINS.BR_DRIVE::hasChanged);
    }

    if (DriverStation.isFMSAttached() || Constants.BURNFLASHES) {
      module.getSteerController().burnFlash();
      module.getDriveController().burnFlash();
    }

    return module;
  }
}
