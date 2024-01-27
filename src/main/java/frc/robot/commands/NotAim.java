// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.enums.SignalSelection;
/** Add your docs here. */
public class NotAim extends Command {
  RobotContainer rc;
int onTarget;

  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();

  /** Creates a new Aim. */
  public NotAim(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTarget = 0;
    // RobotContainer.signalSelection = SignalSelection.Cone_Solid;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rc.shooterSubsystem.isSpeakerShotReady()) {
      RobotContainer.signalSelection = SignalSelection.Cone_Solid;
      onTarget++;
    } else {
      RobotContainer.signalSelection = SignalSelection.Cone_Off;
      onTarget= 0;
    }

    rc.drivetrain.setControl(drive
        .withVelocityX(rc.joystick.getLeftYSmoothed())
        .withVelocityY(rc.joystick.getLeftXSmoothed())
        .withRotationalRate(rc.shooterSubsystem.getSpeakerTurnRate()));

    rc.shooterSubsystem.shooterRotationPID(rc.shooterSubsystem.getDesiredSpeakerWaypoint().getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rc.shooterSubsystem.shooterRotationPID(-0.2);
    RobotContainer.signalSelection = SignalSelection.All_Off;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return onTarget>=20;
  }
}
