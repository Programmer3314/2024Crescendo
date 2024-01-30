// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class GoShoot extends Command {

  private RobotContainer rc;
  private Command pathCommand;

  public GoShoot(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d currentPose = MMField.getBluePose(rc.drivetrain.getState().Pose);
    PathConstraints trajectoryConstraints = new PathConstraints(2, 3, 2 * Math.PI, 4 * Math.PI);
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        currentPose,
        MMField.getBlueWooferApproachPose(),
        MMField.getBlueWooferPose());
    PathPlannerPath path = new PathPlannerPath(bezierPoints,
        trajectoryConstraints,
        new GoalEndState(0, Rotation2d.fromDegrees(180)));
    path.preventFlipping = false;
    pathCommand = AutoBuilder.followPath(path);
    pathCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
