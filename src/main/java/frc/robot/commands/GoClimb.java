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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class GoClimb extends Command {

  private RobotContainer rc;
  private Command pathCommand;
  private Pose2d approachPose;

  public GoClimb(RobotContainer rc, Pose2d approachPose) {
    this.rc = rc;
    this.approachPose = approachPose;
    SmartDashboard.putString("goClimbStatus", "constructed");
    addRequirements(rc.drivetrain);
  }

  // .813 how far back
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rc.navigation.setOneTargetBack(true);
    Pose2d currentPose = MMField.getBluePose(rc.drivetrain.getState().Pose);
    PathConstraints trajectoryConstraints = new PathConstraints(.5, 1, 2 * Math.PI, 4 * Math.PI);
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        currentPose,
        approachPose);
    PathPlannerPath path = new PathPlannerPath(bezierPoints,
        trajectoryConstraints,
        new GoalEndState(0, Rotation2d.fromDegrees(approachPose.getRotation().getDegrees())));
    path.preventFlipping = false;
    pathCommand = AutoBuilder.followPath(path);
    CommandScheduler.getInstance().registerComposedCommands(pathCommand);
    pathCommand.initialize();
    SmartDashboard.putString("goClimbStatus", "initialize");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathCommand.execute();
    SmartDashboard.putString("goClimbStatus", "executed");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
    SmartDashboard.putString("goClimbStatus", interrupted ? "interrupted" : "not interrupted");
    rc.navigation.setOneTargetBack(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putString("goClimbStatus", pathCommand.isFinished() ? "finished" : "not finished");

    return pathCommand.isFinished();
  }
}
