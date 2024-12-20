// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class LowerIntakeChase extends Command {
  /** Creates a new LowerIntakeChase. */
  RobotContainer rc;

  public LowerIntakeChase(RobotContainer rc) {
    this.rc = rc;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rc.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rc.shooterSubsystem.setIntakeFlag(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Chase Lowered Intake",
        Math.abs(rc.shooterSubsystem.getIntakePos() - rc.shooterSubsystem.getIntakeDownPosition()) < .05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(rc.shooterSubsystem.getIntakePos() - rc.shooterSubsystem.getIntakeDownPosition()) < .05;
  }
}
