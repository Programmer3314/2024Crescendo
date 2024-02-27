// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Utility;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;
import frc.robot.commands.Autos.StandardAutoInit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Reign extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;
  String[] pathFileNames;

  /** Creates a new TwoShotAuto. */
  public Reign(RobotContainer rc, String[] pathFileNames) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.pathFileNames = pathFileNames;
  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    for (String pathFile : pathFileNames) {
      cmd.addCommands(
          new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
          new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
          new FollowPathFile(rc, pathFile),
          new ShootAndWait(rc));
    }
    cmd.initialize();
  }
}
