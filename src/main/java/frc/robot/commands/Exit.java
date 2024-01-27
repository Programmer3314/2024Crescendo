// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Exit extends Command {
  Command pathCommand;
  RobotContainer rc;

  /** Creates a new Exit. */
  public Exit(RobotContainer rc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Exit");

    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
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
