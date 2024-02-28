// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Competition;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;
import frc.robot.commands.Autos.StandardAutoInit;
import frc.robot.commands.Autos.Utility.ReignChain;

public class Arabian extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public Arabian(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);

  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new ShootAndWait(rc),
        // new ReignChain(rc, "comp_ab_1", "comp_ab_2"),
        // new ReignChain(rc, "comp_ab_3", "comp_ab_4")
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_ab_1"),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_ab_2"),
        new ShootAndWait(rc),
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_ab_3"),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_ab_4"),
        new ShootAndWait(rc)

    );
    cmd.initialize();
  }
}
