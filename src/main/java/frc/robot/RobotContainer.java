// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DRIVECONSTANTS;
import frc.robot.Constants.VISIONCONSTANTS;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.DriveWithJoysticksEgocentric;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static final Pigeon gyro = null; // new Pigeon();
  public static final DriveTrain driveTrain = new DriveTrain();
  public static final Vision vision = new Vision(VISIONCONSTANTS.CAMERA_NAME);
  public static final Superstructure superstructure = new Superstructure(driveTrain, gyro, vision);

  // OI
  public static final Joystick leftJoystick = new Joystick(DRIVECONSTANTS.LEFT_JOYSTICK_PORT);
  public static final Joystick rightJoystick = new Joystick(DRIVECONSTANTS.RIGHT_JOYSTICK_PORT);

  // temp
  public static final XboxController xbox = new XboxController(5);

  //
  public static final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autonChooser.setDefaultOption("Placeholder", null);
    autonChooser.setDefaultOption("Placeholder", null);
    SmartDashboard.putData(autonChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  public static final void disabledInit() {
    superstructure.haltEverything();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // temp
    new JoystickButton(xbox, Button.kA.value).whenPressed(new DriveWithJoysticksEgocentric(
      driveTrain,
      xbox::getRightX,
      xbox::getRightY,
      xbox::getLeftX
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}
