// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class MustangAuto extends Command {
  RobotContainer rc;
  // TODO: when you are ready to expand this to multiple cycles...
  // consider using SequentialCommandGroup for type of cmd. 
  Command cmd;

  public MustangAuto(RobotContainer rc) {
    this.rc = rc;

    addRequirements(rc.drivetrain);
  }

  @Override
  public void initialize() {
    cmd = Commands.sequence(
        new StandardAutoInit(rc, MMField.currentWooferPose()),
        new MustangAutoCycle(rc, RobotContainer.noteChooser.getSelected(), RobotContainer.shootChooser.getSelected()));
    cmd.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    cmd.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cmd.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cmd.isFinished();
  }
}
