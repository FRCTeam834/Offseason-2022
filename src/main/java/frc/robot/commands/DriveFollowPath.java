package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.PIDGAINS;
import frc.robot.subsystems.DriveTrain;

public class DriveFollowPath extends SequentialCommandGroup {
  /**
   * 
   * @param driveTrain
   * @param trajectory
   * @param robotPoseSupplier
   * @param odometryResetter
   * @param resetOdometry
   */
  public DriveFollowPath(
    DriveTrain driveTrain,
    PathPlannerTrajectory trajectory,
    Supplier<Pose2d> robotPoseSupplier,
    Consumer<Pose2d> odometryResetter,
    boolean resetOdometry
  ) {
    addCommands(
      new InstantCommand(() -> {
        if (resetOdometry) {
          odometryResetter.accept(trajectory.getInitialHolonomicPose());
        }
      }),
      new PPSwerveControllerCommand(
        trajectory,
        robotPoseSupplier,
        driveTrain.getKinematics(),
        PIDGAINS.AUTON_X.generateController(),
        PIDGAINS.AUTON_Y.generateController(),
        PIDGAINS.AUTON_STEER.generateController(),
        driveTrain::setDesiredModuleStates,
        // subsystem requirements
        driveTrain)
    );
  }
}
