// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class PathFindTo extends Command {
  private RobotContainer rc;
  private Command pathfindingCommand;
  private Pose2d targetPose;
  private Supplier<Pose2d> targetPoseSupplier;
  private PathConstraints trajectoryConstraints = new PathConstraints(1, 3,  Math.PI, 2 * Math.PI);
  private double goalEndVelocity;
  private double rotationDelayDistance;

  /** Creates a new FindShoot. */
  public PathFindTo(RobotContainer rc, Pose2d targetPose) {
    this.rc = rc;
    this.targetPose = targetPose;
    this.targetPoseSupplier = this::getTargetPose;

    addRequirements(rc.drivetrain);
  }

  public PathFindTo(RobotContainer rc, Supplier<Pose2d> targetPoseSupplier) {
    this.rc = rc;
    this.targetPoseSupplier = targetPoseSupplier;

    addRequirements(rc.drivetrain);
  }

  private Pose2d getTargetPose() {
    return targetPose;
  }

  public PathFindTo setPathConstraints(PathConstraints pathConstraints) {
    trajectoryConstraints = pathConstraints;
    return this;
  }

  public PathFindTo setGoalEndVelocity(double goalEndVelocity) {
    this.goalEndVelocity = goalEndVelocity;
    return this;
  }

  public PathFindTo setRotationDelayDistance(double rotationDelayDistance) {
    this.rotationDelayDistance = rotationDelayDistance;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathfindingCommand = AutoBuilder.pathfindToPose(
        targetPoseSupplier.get(),
        trajectoryConstraints,
        goalEndVelocity, // Goal end velocity in meters/sec
        rotationDelayDistance // Rotation delay distance in meters. This is how far the robot should travel
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
