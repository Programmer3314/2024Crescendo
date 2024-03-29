// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    addCommands(
        Commands.select(Map.ofEntries(// If we are below a certain point, we need to lower the intake first, but from
                                      // afar we need to prioritize speed
            Map.entry(true, new LowerIntakeChase(rc))),
            () -> rc.navigation.noteBelowThreshold()),
        new ChaseNoteBroken(rc, rc.navigation.noteBelowThreshold()),
        new DriveForwardDistBroken(rc, 0.5, -0.75, true));
  }

  public ChaseAndIntakeBroken(RobotContainer rc, boolean isFast) {
    addCommands(
        Commands.select(Map.ofEntries(
            Map.entry(true, new LowerIntakeChase(rc))),
            () -> rc.navigation.noteBelowThreshold()),
        new ChaseNoteBroken(rc, rc.navigation.noteBelowThreshold()),
        new DriveForwardDistBroken(rc, 0.5, isFast ? -2 : -.75, true));
  }
}
