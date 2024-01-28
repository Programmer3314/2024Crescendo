// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// TODO: review options for handling sequential commands
// Could be handled in the binding itself, either with a Commands.sequential... or as Noted below.
// Or like this...

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootSmove extends SequentialCommandGroup {
  /** Creates a new ShootSmove. */
  RobotContainer rc;

  public ShootSmove(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    addCommands(
        new Exit(rc), new ChaseCone(rc), new FindShoot(rc), new NotAim(rc));
  }
}
