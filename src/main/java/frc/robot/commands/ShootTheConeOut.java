// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ShootTheConeOut extends Command {
  RobotContainer rc;
private int periodic = 0;
  public ShootTheConeOut(RobotContainer rc) {
this.rc = rc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rc.claw.Out();
    rc.claw.openClaw();
    periodic = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    periodic++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.claw.Stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return periodic>50;
  }
}
