// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.libs.SparkMaxController;

public class test extends CommandBase {
  SparkMaxController steerController = new SparkMaxController(1);
  SparkMaxController driveController = new SparkMaxController(5);
  /** Creates a new test. */
  public test() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveController.configVelocityControlP(1.0);
    driveController.configVelocityConversionFactor(1);
    steerController.configPositionControlP(1.0);
    steerController.configPositionConversionFactor(360.0 / 12.8);
    //steerController.setDesiredPosition(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveController.setDesiredVelocity(2);
    //DriverStation.reportWarning(String.format("%f", driveController.getCurrentVelocity()), false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveController.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
