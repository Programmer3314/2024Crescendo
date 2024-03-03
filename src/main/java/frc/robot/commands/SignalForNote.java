// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SignalForNote extends Command {
  /** Creates a new SignalForNote. */
  RobotContainer rc;
  double startTime = 0;
  double runTime = 5;

  public SignalForNote(RobotContainer rc) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rc = rc;
    addRequirements(rc.aBlinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.aBlinkin.gotNote();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentTime = Timer.getFPGATimestamp();

    return currentTime - startTime >= runTime;
  }
}
