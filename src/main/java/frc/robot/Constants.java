// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.libs.PIDGains;
import frc.robot.libs.UnitQuad;
import frc.robot.libs.UnitScaleFunction;
import frc.robot.utilities.TuneablePIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean telemetry = true;
  public static final boolean tuningMode = true;

  /**
   * vvvvvvvvvvvvvvvvvvvvvvvvvv
   * >> !Set to true precomp <<
   * vvvvvvvvvvvvvvvvvvvvvvvvvv
   */
  public static final boolean BURNFLASHES = false;

  // Standard deviations for swerve pose estimator
  public static final Matrix<N3, N1> STATE_STDDEVS  = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0); // [x, y, theta]
  public static final Matrix<N1, N1> LOCAL_STDDEVS  = new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.0); // [theta]
  public static final Matrix<N3, N1> VISION_STDDEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.0, 0.0, 0.0); // [x, y, theta]

  public static final class DRIVECONSTANTS {
    public static final int LEFT_JOYSTICK_PORT = 0;
    public static final int RIGHT_JOYSTICK_PORT = 1;

    // m/s
    public static final double MAX_TRANSLATIONAL_SPEED = 4;
    // degrees per second
    public static final double MAX_STEER_SPEED = 360;

    // Joystick deadzones
    public static final double TRANSLATIONAL_DEADZONE = 0.075;
    public static final double STEER_DEADZONE = 0.075;

    public static final double TRANSLATIONAL_RATELIMIT = 2.0;
    public static final double STEER_RATELIMIT = 4.0;
    public static final UnitScaleFunction STEER_JOYSTICK_SCALING_FUNCTION = new UnitQuad();
    public static final UnitScaleFunction DRIVE_JOYSTICK_SCALING_FUNCTION = new UnitQuad();

    // seconds
    public static final double KEEP_ANGLE_ENABLE_TIME = 0.2;
    // degrees
    public static final double KEEP_ANGLE_ENABLE_TOLERANCE = 0.5;
  }

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
    public static final PIDGains FL_STEER = new TuneablePIDGains("FL_STEER", 1.0);
    public static final PIDGains FL_DRIVE = new PIDGains(1.0);

    public static final PIDGains FR_STEER = new PIDGains(1.0);
    public static final PIDGains FR_DRIVE = new PIDGains(1.0);

    public static final PIDGains BL_STEER = new PIDGains(1.0);
    public static final PIDGains BL_DRIVE = new PIDGains(1.0);

    public static final PIDGains BR_STEER = new PIDGains(1.0);
    public static final PIDGains BR_DRIVE = new PIDGains(1.0);

    public static final PIDGains KEEP_ANGLE = new PIDGains(1.0);

    public static final PIDGains AUTON_X = new PIDGains(1.0);
    public static final PIDGains AUTON_Y = new PIDGains(1.0);
    public static final PIDGains AUTON_STEER = new PIDGains(1.0);
  }

  public static final class FFGAINS {
    public static final SimpleMotorFeedforward FL_STEER_FF = new SimpleMotorFeedforward(0.0, 0.0);
    public static final SimpleMotorFeedforward FL_DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0);

    public static final SimpleMotorFeedforward FR_STEER_FF = new SimpleMotorFeedforward(0.0, 0.0);
    public static final SimpleMotorFeedforward FR_DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0);

    public static final SimpleMotorFeedforward BL_STEER_FF = new SimpleMotorFeedforward(0.0, 0.0);
    public static final SimpleMotorFeedforward BL_DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0);

    public static final SimpleMotorFeedforward BR_STEER_FF = new SimpleMotorFeedforward(0.0, 0.0);
    public static final SimpleMotorFeedforward BR_DRIVE_FF = new SimpleMotorFeedforward(0.0, 0.0);
  }

  public static final class SWERVEMODULECONSTANTS {
    public static final double MAX_SPEED = 4.5; // m/s
    public static final double WHEEL_DIAMETER = 0.1; // m
    public static final double DRIVE_GEAR_RATIO = 8.14;
    public static final double STEER_GEAR_RATIO = 12.8;
    // degrees
    public static final double POSITION_CONVERSION_FACTOR = 360 / STEER_GEAR_RATIO;
    // meters per second
    public static final double VELOCITY_CONVERSION_FACTOR = 60 * Math.PI * WHEEL_DIAMETER / DRIVE_GEAR_RATIO;

    // Some modules are inverted, add 180 offset to artifically invert
    public static final double FL_MAGNET_OFFSET = 0.0 + 180.0;
    public static final double FR_MAGNET_OFFSET = 0.0;
    public static final double BL_MAGNET_OFFSET = 0.0;
    public static final double BR_MAGNET_OFFSET = 0.0 + 180.0;
  }

  public static final class DRIVETRAINCONSTANTS {
    public static final Translation2d FLM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d FRM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BLM_POS = new Translation2d(0.0, 0.0);
    public static final Translation2d BRM_POS = new Translation2d(0.0, 0.0);

    // Set as null if no idle configuration is wanted
    public static final SwerveModuleState[] IDLE_MODULE_CONFIGURATION = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };
  }

  public static final class VISIONCONSTANTS {
    public static final String CAMERA_NAME = "Einstein2023";
    // Transform from camera to robot center (center at base of robot)
    public static final Transform3d CAMERA_POS = new Transform3d(
      new Translation3d(0.0, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, 0.0)
    );
  }
}
