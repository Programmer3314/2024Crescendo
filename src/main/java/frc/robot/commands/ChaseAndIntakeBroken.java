// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChaseAndIntakeBroken extends SequentialCommandGroup {
  RobotContainer rc;
  public static boolean abortDrive;

  /** Creates a new ChaseAndIntake. */
  public ChaseAndIntakeBroken(RobotContainer rc) {
    this.rc = rc;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LowerIntakeChase(rc),
        new ChaseNoteBroken(rc),
        new DriveForwardDistBroken(rc, 0.5, -0.75, true));
  }

  public ChaseAndIntakeBroken(RobotContainer rc, boolean isFast) {
    addCommands(
        new LowerIntakeChase(rc),
        new ChaseNoteBroken(rc),
        new DriveForwardDistBroken(rc, 0.5, isFast ? -2 : -.75, true));
  }
}
