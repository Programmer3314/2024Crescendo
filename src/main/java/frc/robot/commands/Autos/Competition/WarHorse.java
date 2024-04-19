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
import frc.robot.commands.ShootAndWaitForced;
import frc.robot.commands.SitShootAndWait;
import frc.robot.commands.SpinUpForAutoShot;
import frc.robot.commands.Autos.StandardAutoInit;

public class WarHorse extends MMDeferredCommand<SequentialCommandGroup> {
    RobotContainer rc;

    /** Creates a new TwoShotAuto. */
    public WarHorse(RobotContainer rc) {
        this.rc = rc;
        addRequirements(rc.drivetrain);

        cmd = new SequentialCommandGroup();
        cmd.addCommands(
                new StandardAutoInit(rc, MMField.currentWooferPose())
                        .setPipeLine(0, 0, 0),
                new SitShootAndWait(rc),
                new SpinUpForAutoShot(rc, "warhorse_1"),
                new ChaseAndIntakeBroken(rc),
                new ShootAndWaitForced(rc, true),

                new FollowPathFile(rc, "comp_wh_2"),
                new ChaseAndIntakeBroken(rc),
                new SpinUpForAutoShot(rc, "warhorse_2"),
                new FollowPathFile(rc, "comp_wh_3"),
                new ShootAndWaitForced(rc, true),

                new SpinUpForAutoShot(rc, "warhorse_3"),
                new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
                new FollowPathFile(rc, "comp_wh_4"),
                new DriveForwardDistBroken(rc, .5, -1),
                new ShootAndWaitForced(rc, true));
    }

    @Override
    public void initialize() {
        cmd.initialize();
    }
}
