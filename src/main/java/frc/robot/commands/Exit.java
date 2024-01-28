// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

// TODO: rework this followPathFromFile
// The parameter should be a Supplier<String> 
// so that the value can be determined at activation
// I suspect atht two constructors can be created 
// so that the value can be either a supplier or a simple string

public class Exit extends Command {
  Command pathCommand;
  RobotContainer rc;

  public Exit(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  @Override
  public void initialize() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Exit");

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
