// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;

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
  }

  public static final class PIDS {
    public static final class SwerveModulePIDS {
      public static final double FL_STEER_kP = 0.0;
      public static final double FL_STEER_kI = 0.0;
      public static final double FL_STEER_kD = 0.0;

      public static final double FR_STEER_kP = 0.0;
      public static final double FR_STEER_kI = 0.0;
      public static final double FR_STEER_kD = 0.0;

      public static final double BL_STEER_kP = 0.0;
      public static final double BL_STEER_kI = 0.0;
      public static final double BL_STEER_kD = 0.0;

      public static final double BR_STEER_kP = 0.0;
      public static final double BR_STEER_kI = 0.0;
      public static final double BR_STEER_kD = 0.0;
    }
  }

  public static final class SwerveModuleConstants {
    public static final Translation2d FL_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d FR_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BL_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BR_POS = new Translation2d(0.0, 0.0);
    
    // !Convert to deg/s
    public static final double DRIVE_POS_CONVERSION_FACTOR = 0;
    // !Convert to m/s
    public static final double DRIVE_VEL_CONVERSION_FACTOR = 0;

    public static final double AT_VELOCITY_TOLERANCE = 0.0;
    public static final double AT_POSITION_TOLERANCE = 0.0;
  }
}
