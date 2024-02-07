// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StageSideAuto extends SequentialCommandGroup {
  /** Creates a new StageSideAuto. */
  RobotContainer rc;

  public StageSideAuto(RobotContainer rc) {
    this.rc = rc;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShootAndWait(rc), // for shoot first before moving
        new ParallelCommandGroup(
          new FollowPathFile(rc, "StageSide1")
          // ,new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true))
        ),
        new ShootAndWait(rc),
        new FollowPathFile(rc, "StageSide2"),
        new ShootAndWait(rc),
        new FollowPathFile(rc, "StageSide3"));
  }
}