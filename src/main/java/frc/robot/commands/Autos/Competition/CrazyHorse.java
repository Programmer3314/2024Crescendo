// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Competition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.ChaseAndIntakeBroken;
import frc.robot.commands.Delay;
import frc.robot.commands.DriveForwardDistBroken;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;
import frc.robot.commands.ShootAndWaitForced;
import frc.robot.commands.ShootAndWaitRegular;
import frc.robot.commands.SpinUpForAutoShot;
import frc.robot.commands.WaitToIndexed;
import frc.robot.commands.Autos.StandardAutoInit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CrazyHorse extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public CrazyHorse(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new SpinUpForAutoShot(rc, "crazyhorse_1"),
        new FollowPathFile(rc, "comp_ch_1"),

        new ShootAndWaitForced(rc, true),

        new SpinUpForAutoShot(rc, "crazyhorse_2"),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new DriveForwardDistBroken(rc, .5, -1),

        // new FollowPathFile(rc, "comp_ch_1"),
        new WaitToIndexed(rc),
        new ShootAndWaitForced(rc),

        new SpinUpForAutoShot(rc, "crazyhorse_3"),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_ch_2"),
        // new DriveForwardDistBroken(rc, 0.5, -.75),
        // new ShootAndWaitForced(rc),

        // new SpinUpForAutoShot(rc, "crazyhorse_4"),
        // new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_ch_3"),
        // new WaitToIndexed(rc),
        new ShootAndWaitForced(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new SpinUpForAutoShot(rc, "crazyhorse_4"),
        new DriveForwardDistBroken(rc, .75, -1),
        new WaitToIndexed(rc),
        new ShootAndWaitForced(rc),

        new FollowPathFile(rc, "comp_ch_4"),
        new ChaseAndIntakeBroken(rc, true),

        new SpinUpForAutoShot(rc, "crazyhorse_5"),
        new FollowPathFile(rc, "comp_ch_5"),
        new Delay(rc, 20),
        new ShootAndWaitForced(rc));

  }

  @Override
  public void initialize() {
    cmd.initialize();
  }
}
