// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class FindShoot extends Command {
  RobotContainer rc;
  Command pathfindingCommand;

  /** Creates a new FindShoot. */
  public FindShoot(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d targetPose = new Pose2d(1.6, 5.5, Rotation2d.fromDegrees(0));
    PathConstraints trajectoryConstraints = new PathConstraints(1, 3, 2 * Math.PI, 4 * Math.PI);
    pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPose,
        trajectoryConstraints,
        0.0, // Goal end velocity in meters/sec
        0.2 // Rotation delay distance in meters. This is how far the robot should travel
            // before attempting to rotate.
    );
    pathfindingCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathfindingCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathfindingCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathfindingCommand.isFinished();
  }
}
