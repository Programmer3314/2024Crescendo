// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class LaserBeam extends Command {
  /** Creates a new LaserBeam. */
  RobotContainer rc;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  double startTime;
  double totalBeamTime;
  double timeOutAfterShot = .3;
  double totalTimeOutTime;
  int startShootCounter;
  int startLightBeamCounter;

  public LaserBeam(RobotContainer rc) {
    this.rc = rc;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startLightBeamCounter = rc.shooterSubsystem.getLightBeamCounter();
    startShootCounter = rc.shooterSubsystem.getShotCounter();
    totalBeamTime = rc.navigation.getFullBeamTime();
    totalTimeOutTime = totalBeamTime + timeOutAfterShot;
    rc.navigation.setUpdatePredictedPose(false);
    startTime = Timer.getFPGATimestamp();
    rc.shooterSubsystem.setRunBeam(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Timer.getFPGATimestamp() - startTime >= totalBeamTime)
        && startLightBeamCounter == rc.shooterSubsystem.getLightBeamCounter()) {
      rc.shooterSubsystem.setLightBeam(true);
    }

    rc.drivetrain.setControl(drive
        .withVelocityX(rc.navigation.getAveragePredictedX())
        .withVelocityY(rc.navigation.getAveragePredictedY())
        .withRotationalRate(rc.shooterSubsystem.getPredictedSpeakerTurnRate()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.navigation.setUpdatePredictedPose(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime >= totalTimeOutTime)
        || rc.shooterSubsystem.getShotCounter() > startShootCounter;
  }
}
