// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class SetColor extends Command {
  RobotContainer rc;
  String status;
  double rbgValue;

  public SetColor(RobotContainer rc, String status) {
    this.rc = rc;
    this.status = status;
    addRequirements(rc.aBlinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (status) {
      case "Intake":
        rbgValue = -.07;
        break;
      case "Shoot":
        rbgValue = -.17;
        break;
      case "Gottem":
        rbgValue = .77;
        break;
      default:
        rbgValue = .77;
        break;
    }

  }

  public void execute() {
    rc.aBlinkin.controlBlink(rbgValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
