// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class Delay extends Command {
  RobotContainer rc;
  private int delay;
  private int delayCount;
  private boolean aim;
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  /** Creates a new Delay. */
  public Delay(RobotContainer rc, int delay) {
    this.rc =rc;
    this.delay = delay;
    aim = false;
    // Use addRequirements() here to declare subsystem dependencies.
  }
    public Delay(RobotContainer rc, int delay, boolean aim) {
    this.rc =rc;
    this.delay = delay;
    this.aim = aim;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delayCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(aim){
      rc.drivetrain.setControl(drive
      .withVelocityX(0)
      .withVelocityY(0)
      .withRotationalRate(rc.shooterSubsystem.getSpeakerTurnRate()));
      }
    delayCount++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return delayCount>= delay;
  }
}
