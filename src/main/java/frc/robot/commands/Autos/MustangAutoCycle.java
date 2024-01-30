// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import org.ejml.equation.Sequence;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.NotAim;
import frc.robot.commands.PathFindTo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MustangAutoCycle extends Command {
  RobotContainer rc;
  Pose2d notePose;
  Pose2d shooterPose;
  Command cmd;

  /** Creates a new PaskackAuto. */
  public MustangAutoCycle(RobotContainer rc, Pose2d notePose, Pose2d shooterPose) {
    this.rc = rc;
    this.notePose = notePose;
    this.shooterPose = shooterPose;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  @Override
  public void initialize() {
    cmd = Commands.sequence(
        new PathFindTo(rc, notePose),
        new DriveForwardDist(rc, .125, .5),
        new PathFindTo(rc, shooterPose),
        new NotAim(rc));
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
