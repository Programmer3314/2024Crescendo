// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class StartClimb extends Command {
  RobotContainer rc;
  int idleCounter =0;
  /** Creates a new GoClimb. */
  public StartClimb(RobotContainer rc) {
    this.rc = rc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    idleCounter = rc.climber.getIdleCounter();
    rc.climber.setClimbFlag(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  // TODO: If Interupted, set on AbortClimb flag. 
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   return rc.climber.getIdleCounter()> idleCounter;

  
  }
}
