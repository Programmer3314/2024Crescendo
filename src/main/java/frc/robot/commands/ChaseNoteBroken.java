// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMPIDController;

public class ChaseNoteBroken extends Command {
  RobotContainer rc;
  MMPIDController rotationPIDController;
  MMPIDController yPIDController;
  // double targetX = 233;
  // double targetY = 308;
  double targetX = 0;
  double targetY = 10;
  int cyclesWithoutNote;
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  boolean hasHadNote;

  public ChaseNoteBroken(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    rotationPIDController = new MMPIDController(.05, 0, 0, 3.0 / 2.0, 0.5, false);
  }

  public ChaseNoteBroken(RobotContainer rc, boolean noteBelowThreshold) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    rotationPIDController = new MMPIDController(noteBelowThreshold ? .05 : .15, 0, 0.005, 3.0 / 2.0, 0.25, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cyclesWithoutNote = 0;
    // rc.shooterSubsystem.setIntakeFlag(true);TODO: BABY MODE
    rotationPIDController.initialize(targetX);
    yPIDController = new MMPIDController(1.0 / 10, 0, 0, 3.0 / 2.0, 0.25, false);// 100
    yPIDController.initialize(targetY);
    hasHadNote = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rc.navigation.hasNoteTarget()) {
      hasHadNote = true;
      cyclesWithoutNote = 0;
    } else {
      cyclesWithoutNote++;
    }
    double driveRotationVelocity = rotationPIDController.execute(rc.navigation.getNoteX());
    double driveYVelocity = yPIDController.execute(rc.navigation.getNoteY());

    if (Math.abs(rc.navigation.getNoteY() - targetY) < 5
        && Math.abs(rc.navigation.getNoteX() - targetX) < 5) {
      driveRotationVelocity = 0;
      driveYVelocity = 0;
    }
    SmartDashboard.putNumber("drive X VEL", driveRotationVelocity);
    rc.drivetrain.setControl(drive
        .withVelocityX(driveYVelocity)
        .withVelocityY(0)
        .withRotationalRate(driveRotationVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    ChaseAndIntakeBroken.abortDrive = !hasHadNote;

    // return (Math.abs(rc.navigation.getNoteY() - targetY) < 15
    // && Math.abs(rc.navigation.getNoteX() - targetX) < 15);
    // // || !rc.shooterSubsystem.getIntakeBreakbeam()
    // // || cyclesWithoutNote > 15;BABYMODE
    return false;
  }
}
