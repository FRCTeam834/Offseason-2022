// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVECONSTANTS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.libs.MathPlus;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class DriveWithJoysticks extends CommandBase {
  // Subsystem requirements
  private DriveTrain driveTrain;
  private Pigeon gyro;

  private DoubleSupplier xRaw;
  private DoubleSupplier yRaw;
  private DoubleSupplier steerRaw;

  private PIDController keepAnglePIDController = PIDGAINS.KEEP_ANGLE.generateController();
  private double keepAngle;
  private Timer timeSinceLastTurn = new Timer();

  // Limits acceleration
  private SlewRateLimiter xRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter yRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter steerRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.STEER_RATELIMIT);

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
    DriveTrain driveTrain,
    Pigeon gyro,
    DoubleSupplier xRaw,
    DoubleSupplier yRaw,
    DoubleSupplier steerRaw
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.gyro = gyro;

    this.xRaw = xRaw;
    this.yRaw = yRaw;
    this.steerRaw = steerRaw;

    keepAnglePIDController.enableContinuousInput(0, 360);

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timeSinceLastTurn.reset();
    timeSinceLastTurn.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // raw value -> deadzone -> scaling -> ratelimit
    double xInput = xRateLimiter.calculate(
      DRIVECONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(xRaw.getAsDouble(), DRIVECONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double yInput = yRateLimiter.calculate(
      DRIVECONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(yRaw.getAsDouble(), DRIVECONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double steerInput = steerRateLimiter.calculate(
      DRIVECONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(steerRaw.getAsDouble(), DRIVECONSTANTS.STEER_DEADZONE)
      )
    );

    // !Note: Normalization is handled by desaturateWheelSpeeds
    double vx = xInput * SWERVEMODULECONSTANTS.MAX_SPEED;
    double vy = yInput * SWERVEMODULECONSTANTS.MAX_SPEED;
    double omega = steerInput * DRIVECONSTANTS.MAX_STEER_SPEED;

    if (omega != 0) {
      timeSinceLastTurn.reset();
    }

    if(timeSinceLastTurn.get() < DRIVECONSTANTS.KEEP_ANGLE_ENABLE_TIME) {
      // Allow time for robot to finish rotation
      keepAngle = gyro.getYaw();
    } else {
      if (vx != 0 || vy != 0) {
        // Don't perform keep angle if there is no intention to move
        omega = keepAnglePIDController.calculate(gyro.getYaw(), keepAngle);
      }
    }

    if (vx == 0 && vy == 0 && omega == 0) {
      driveTrain.setIdleModuleStates();
    } else {
      driveTrain.driveFieldCentric(vx, vy, omega, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setIdleModuleStates();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
