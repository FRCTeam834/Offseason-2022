// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;

public class DriveWithJoysticks extends CommandBase {
  private DriveTrain driveTrain;
  private Pigeon gyro;
  
  private Joystick steerJoystick;
  private Joystick driveJoystick;

  private PIDController keepAnglePIDController;
  private double keepAngle;

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
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
