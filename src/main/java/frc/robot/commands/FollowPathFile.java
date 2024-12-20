// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class FollowPathFile extends Command {
  Command pathCommand;
  RobotContainer rc;
  String pathFile;
  Supplier<String> pathFileSupplier;

  public FollowPathFile(RobotContainer rc, String pathFile) {
    this.rc = rc;
    this.pathFile = pathFile;
    this.pathFileSupplier = this::getPathFile;
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathFileSupplier.get());
    pathCommand = AutoBuilder.followPath(path);

    addRequirements(rc.drivetrain);
  }

  public FollowPathFile(RobotContainer rc, Supplier<String> pathFileSupplier) {
    this.rc = rc;
    this.pathFileSupplier = pathFileSupplier;
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathFileSupplier.get());
    pathCommand = AutoBuilder.followPath(path);

    addRequirements(rc.drivetrain);
  }

  private String getPathFile() {
    return pathFile;
  }

  @Override
  public void initialize() {

    pathCommand.initialize();
  }

  @Override
  public void execute() {
    pathCommand.execute();
  }

  @Override
  public void end(boolean interrupted) {
    pathCommand.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
