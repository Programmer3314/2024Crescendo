// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.NotAim;
import frc.robot.commands.PathFindTo;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MustangAuto extends Command {
  /** Creates a new MustangAuto. */
  RobotContainer rc;
  Command cmd;

  public MustangAuto(RobotContainer rc) {
    this.rc = rc;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
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
