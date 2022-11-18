// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVECONSTANTS;
import frc.robot.Constants.SWERVEMODULECONSTANTS;
import frc.robot.libs.MathPlus;
import frc.robot.subsystems.DriveTrain;

public class DriveWithJoysticksEgocentric extends CommandBase {

  private DriveTrain driveTrain;
  private DoubleSupplier xRaw;
  private DoubleSupplier yRaw;
  private DoubleSupplier steerRaw;

  // Limits acceleration
  private SlewRateLimiter xRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter yRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.TRANSLATIONAL_RATELIMIT);
  private SlewRateLimiter steerRateLimiter = new SlewRateLimiter(DRIVECONSTANTS.STEER_RATELIMIT);

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticksEgocentric(
    DriveTrain driveTrain,
    DoubleSupplier xRaw,
    DoubleSupplier yRaw,
    DoubleSupplier steerRaw
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;

    this.xRaw = xRaw;
    this.yRaw = yRaw;
    this.steerRaw = steerRaw;

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
      DRIVECONSTANTS.DRIVE_JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(xRaw.getAsDouble(), DRIVECONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double yInput = yRateLimiter.calculate(
      DRIVECONSTANTS.DRIVE_JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(yRaw.getAsDouble(), DRIVECONSTANTS.TRANSLATIONAL_DEADZONE)
    ));

    double steerInput = steerRateLimiter.calculate(
      DRIVECONSTANTS.STEER_JOYSTICK_SCALING_FUNCTION.calculate(
        MathPlus.applyDeadzone(steerRaw.getAsDouble(), DRIVECONSTANTS.STEER_DEADZONE)
      )
    );

    // !Note: Normalization is handled by desaturateWheelSpeeds
    // This means we can multiply each xy component by moduleMaxSpeed without issues
    double vx = xInput * SWERVEMODULECONSTANTS.MAX_SPEED;
    double vy = yInput * SWERVEMODULECONSTANTS.MAX_SPEED;
    double omega = steerInput * DRIVECONSTANTS.MAX_STEER_SPEED;

    if (vx == 0 && vy == 0 && omega == 0) {
      driveTrain.setIdleModuleStates();
    } else {
      driveTrain.driveRobotCentricOpenLoop(vx, vy, omega);
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
