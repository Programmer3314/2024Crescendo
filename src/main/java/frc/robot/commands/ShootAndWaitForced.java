// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMWaypoint;

public class ShootAndWaitForced extends Command {
  RobotContainer rc;
  double runIndexerStartTime;
  boolean firstTimeShooting = true;
  double waitTime = 1;
  int startShootCounter;
  boolean aim;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  /** Creates a new ShootAndWait. */
  public ShootAndWaitForced(RobotContainer rc) {
    this.rc = rc;
    aim = true;
    addRequirements(rc.drivetrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShootAndWaitForced(RobotContainer rc, boolean aim) {
    this.rc = rc;
    this.aim = aim;
    addRequirements(rc.drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    startShootCounter = rc.shooterSubsystem.getShotCounter();

    rc.shooterSubsystem.setReadyToAutoShoot(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rc.shooterSubsystem.setReadyToAutoShoot(true);

    if (aim) {
      rc.drivetrain.setControl(drive
          .withVelocityX(0)
          .withVelocityY(0)
          .withRotationalRate(rc.shooterSubsystem.getSpeakerTurnRate()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.shooterSubsystem.setReadyToAutoShoot(false);
    rc.shooterSubsystem.setAutoForce(false);
    rc.shooterSubsystem.setAimFlag(false);
    rc.shooterSubsystem.setShootFlag(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return rc.shooterSubsystem.getShotCounter() > startShootCounter;
    // return false;
  }
}
