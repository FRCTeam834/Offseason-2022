// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class CANIDS {
    public static final int FL_STEER_ID    = 1;
    public static final int FL_DRIVE_ID    = 2;
    public static final int FL_CANCODER_ID = 3;
    public static final int FR_STEER_ID    = 4;
    public static final int FR_DRIVE_ID    = 5;
    public static final int FR_CANCODER_ID = 6;
    public static final int BL_STEER_ID    = 7;
    public static final int BL_DRIVE_ID    = 8;
    public static final int BL_CANCODER_ID = 9;
    public static final int BR_STEER_ID    = 10;
    public static final int BR_DRIVE_ID    = 11;
    public static final int BR_CANCODER_ID = 12;
    
    public static final int PIGEON2_ID     = 13;
  }

  public static final class PIDGAINS {
    public static final double FL_STEER_kP  = 0.0;
    public static final double FL_STEER_kI  = 0.0;
    public static final double FL_STEER_kD  = 0.0;
    public static final double FL_STEER_kFF = 0.0;

    public static final double FL_DRIVE_kP  = 0.0;
    public static final double FL_DRIVE_kI  = 0.0;
    public static final double FL_DRIVE_kD  = 0.0;
    public static final double FL_DRIVE_kFF = 0.0;

    public static final double FR_STEER_kP  = 0.0;
    public static final double FR_STEER_kI  = 0.0;
    public static final double FR_STEER_kD  = 0.0;
    public static final double FR_STEER_kFF = 0.0;

    public static final double FR_DRIVE_kP  = 0.0;
    public static final double FR_DRIVE_kI  = 0.0;
    public static final double FR_DRIVE_kD  = 0.0;
    public static final double FR_DRIVE_kFF = 0.0;

    public static final double BL_STEER_kP  = 0.0;
    public static final double BL_STEER_kI  = 0.0;
    public static final double BL_STEER_kD  = 0.0;
    public static final double BL_STEER_kFF = 0.0;

    public static final double BL_DRIVE_kP  = 0.0;
    public static final double BL_DRIVE_kI  = 0.0;
    public static final double BL_DRIVE_kD  = 0.0;
    public static final double BL_DRIVE_kFF = 0.0;

    public static final double BR_STEER_kP  = 0.0;
    public static final double BR_STEER_kI  = 0.0;
    public static final double BR_STEER_kD  = 0.0;
    public static final double BR_STEER_kFF = 0.0;

    public static final double BR_DRIVE_kP  = 0.0;
    public static final double BR_DRIVE_kI  = 0.0;
    public static final double BR_DRIVE_kD  = 0.0;
    public static final double BR_DRIVE_kFF = 0.0;
  }

  public static final class SWERVEMODULECONSTANTS {
    // meters per second
    public static final double MAX_SPEED = 0.0;
    public static final double WHEEL_DIAMETER = 0.0;
    public static final double GEAR_RATIO = 0.0;
    // degrees
    public static final double POSITION_CONVERSION_FACTOR = 360 * Math.PI * WHEEL_DIAMETER / GEAR_RATIO;
    // meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = 60 * Math.PI * WHEEL_DIAMETER;

    public static final double FL_MAGNET_OFFSET = 0.0;
    public static final double FR_MAGNET_OFFSET = 0.0;
    public static final double BL_MAGNET_OFFSET = 0.0;
    public static final double BR_MAGNET_OFFSET = 0.0;
  }

  public static final class DRIVETRAINCONSTANTS {
    public static final Translation2d FLM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d FRM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BLM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BRM_POS = new Translation2d(0.0, 0.0);

    public static final SwerveModuleState[] IDLE_MODULE_CONFIGURATION = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
  }
}
