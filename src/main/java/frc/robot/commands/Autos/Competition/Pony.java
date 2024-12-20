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
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;
import frc.robot.commands.SitShootAndWait;
import frc.robot.commands.SpinUpForAutoShot;
import frc.robot.commands.Autos.StandardAutoInit;

public class Pony extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public Pony(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);

    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.getCurrentWooferNonHumanPlayerPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new SitShootAndWait(rc),
        // new ReignChain(rc, "comp_ab_1", "comp_ab_2"),
        // new ReignChain(rc, "comp_ab_3", "comp_ab_4")
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_pony_1"),
        // new ChaseAndIntakeBroken(rc),
        new ChaseAndIntakeBroken(rc, true),

        new SpinUpForAutoShot(rc, "pony_2"),
        // new ShootAndWait(rc),

        // new InstantCommand(() -> rc.shooterSubsystem.setAutoForce(true)),
        // new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_pony_2"),

        new Delay(rc, 50, true),
        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_pony_3"),
        // new ChaseAndIntakeBroken(rc),
        new ChaseAndIntakeBroken(rc, true),
        // new InstantCommand(() -> rc.shooterSubsystem.setAutoForce(true)),

        new SpinUpForAutoShot(rc, "pony_3"),
        // new ShootAndWait(rc),
        
        // new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_pony_4"),
        new Delay(rc, 25, true),//50
        new ShootAndWait(rc));
  }

  @Override
  public void initialize() {
    cmd.initialize();
  }
}
