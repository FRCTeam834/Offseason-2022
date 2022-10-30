// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVERCONSTANTS;
import frc.robot.Constants.PIDGAINS;
import frc.robot.libs.MathPlus;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class DriveWithJoysticks extends CommandBase {
  // Subsystem requirements
  private DriveTrain driveTrain;
  private Pigeon gyro;

  private Joystick steerJoystick;
  private Joystick driveJoystick;

  private PIDController keepAnglePIDController = PIDGAINS.KEEP_ANGLE.generateController();
  private double keepAngle;

  // Limits acceleration
  private SlewRateLimiter xRateLimiter = new SlewRateLimiter(DRIVERCONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter yRateLimiter = new SlewRateLimiter(DRIVERCONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter steerRateLimiter = new SlewRateLimiter(DRIVERCONSTANTS.STEER_RATELIMIT);

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(
    DriveTrain driveTrain,
    Pigeon gyro,
    Joystick steerJoystick,
    Joystick driveJoystick
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.gyro = gyro;

    this.steerJoystick = steerJoystick;
    this.driveJoystick = driveJoystick;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // raw value -> deadzone -> scaling -> ratelimit
    double xInput = xRateLimiter.calculate(
      DRIVERCONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(driveJoystick.getX(), DRIVERCONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double yInput = yRateLimiter.calculate(
      DRIVERCONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(driveJoystick.getY(), DRIVERCONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double steerInput = steerRateLimiter.calculate(
      DRIVERCONSTANTS.JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(steerJoystick.getX(), DRIVERCONSTANTS.STEER_DEADZONE)
      )
    );

    double theta = Math.atan2(yInput, xInput);
    // sqrt(2) / hypot(x, y) = % max speed; where sqrt(2) is hypot of 1 by 1 triangle
    double speed = 1.414214 / Math.hypot(xInput, yInput) * DRIVERCONSTANTS.MAX_TRANSLATIONAL_SPEED;

    double vx = speed * Math.cos(theta);
    double vy = speed * Math.sin(theta);
    double omega = DRIVERCONSTANTS.MAX_STEER_SPEED * steerInput;

    if (vx == 0 && vy == 0 && omega == 0) {
      driveTrain.setIdleModuleStates();
    } else {
      driveTrain.driveFieldCentric(vx, vy, omega, true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
