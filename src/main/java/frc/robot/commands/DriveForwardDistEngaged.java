// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class DriveForwardDistEngaged extends Command {
  RobotContainer rc;
  double distance;
  SwerveRequest.RobotCentric drive;
  double velocity;
  double distanceTraveled;
  Translation2d initialPosition;
  boolean robotStop = true;

  public DriveForwardDistEngaged(RobotContainer rc, double distance, double velocity) {
    this.rc = rc;
    this.distance = distance;
    this.drive = new SwerveRequest.RobotCentric();
    this.velocity = velocity;

    addRequirements(rc.drivetrain);
  }

  public DriveForwardDistEngaged setRobotStop(boolean robotStop) {
    this.robotStop = robotStop;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPosition = rc.drivetrain.getState().Pose.getTranslation();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rc.drivetrain.setControl(drive.withVelocityX(velocity));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (robotStop) {
      rc.drivetrain.setControl(drive.withVelocityX(0.0));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    distanceTraveled = rc.drivetrain.getState().Pose.getTranslation().minus(initialPosition).getNorm();
    return distance <= distanceTraveled || rc.climber.climbEngaged();
  }
}
