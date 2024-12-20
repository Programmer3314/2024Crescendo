// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Aim extends Command {
  RobotContainer rc;

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  public Aim(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rc.shooterSubsystem.setAimFlag(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rc.shooterSubsystem.aimToSpeaker();

    
    rc.drivetrain.setControl(drive
        .withVelocityX(rc.driverController.getLeftYSmoothed() * Robot.resetDriverValue)
        .withVelocityY(rc.driverController.getLeftXSmoothed() * Robot.resetDriverValue)
        .withRotationalRate(rc.shooterSubsystem.getSpeakerTurnRate()));

    // rc.drivetrain.setControl(drive
    // .withVelocityX(rc.driverController.getLeftYSmoothed() *
    // Robot.resetDriverValue)
    // .withVelocityY(rc.driverController.getLeftXSmoothed() *
    // Robot.resetDriverValue)
    // .withRotationalRate(rc.shooterSubsystem.getPredictedSpeakerTurnRate()));
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
