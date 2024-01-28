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
    this.pathFile = pathFile;
    this.rc = rc;
    addRequirements(rc.drivetrain);
    this.pathFileSupplier = this::getPathFile;
    
  }

  public FollowPathFile(RobotContainer rc, Supplier<String> pathFileSupplier) {
    this.rc = rc;
    
    addRequirements(rc.drivetrain);
    this.pathFileSupplier = pathFileSupplier;
    
  }
  private String getPathFile(){
    return pathFile;
  }
  @Override
  public void initialize() {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathFileSupplier.get());

    pathCommand = AutoBuilder.followPath(path);
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
