// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMTurnPIDController;

public class DynamicChuckAim extends Command {
  RobotContainer rc;

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  MMTurnPIDController turnPIDController = new MMTurnPIDController(true);

  public DynamicChuckAim(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rc.shooterSubsystem.setDynamicChuckFlag(true);
    turnPIDController.initialize(MMField.getCurrentChuckRotation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rc.shooterSubsystem.setDynamicChuckFlag(true);
    Rotation2d rotation = rc.drivetrain.getState().Pose.getRotation();
    SmartDashboard.putNumber("Current Rotation", rotation.getDegrees());

    rc.drivetrain.setControl(drive
        .withVelocityX(rc.driverController.getLeftYSmoothed() * Robot.resetDriverValue)
        .withVelocityY(rc.driverController.getLeftXSmoothed() * Robot.resetDriverValue)
        .withRotationalRate(turnPIDController.execute(rotation)));

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
    rc.shooterSubsystem.setDynamicChuckFlag(false);
    rc.shooterSubsystem.resetStateMachine();
    rc.shooterSubsystem.stopShooterMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
