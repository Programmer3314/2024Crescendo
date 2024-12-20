// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Competition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.ChaseAndIntakeBroken;
import frc.robot.commands.Delay;
import frc.robot.commands.DriveForwardDistBroken;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;
import frc.robot.commands.SitShootAndWait;
import frc.robot.commands.WaitToIndexed;
import frc.robot.commands.Autos.StandardAutoInit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HorseShoe extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public HorseShoe(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new SitShootAndWait(rc),
        // new Reign(rc, new String[] { "comp_hs_1", "comp_hs_2", "comp_hs_3" })
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        // new FollowPathFile(rc, "comp_hs_1"),
        // new DriveForwardDistBroken(rc, .5, -1),
        new ChaseAndIntakeBroken(rc),

        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_hs_2"),
        new DriveForwardDistBroken(rc, .5, -1),
        new Delay(rc, 15),
        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_hs_3"),
        new DriveForwardDistBroken(rc, .5, -1),

        new Delay(rc, 15),
        new ShootAndWait(rc));
  }

  @Override
  public void initialize() {
    cmd.initialize();
  }
}
