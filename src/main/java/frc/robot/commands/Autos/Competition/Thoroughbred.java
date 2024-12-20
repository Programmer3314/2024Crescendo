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
import frc.robot.commands.ShootAndWaitForced;
import frc.robot.commands.ShootAndWaitRegular;
import frc.robot.commands.SitShootAndWait;
import frc.robot.commands.SpinUpForAutoShot;
import frc.robot.commands.Autos.StandardAutoInit;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Thoroughbred extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public Thoroughbred(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.getCurrentWooferHumanPlayerPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new SitShootAndWait(rc),
        // new Reign(rc, new String[] { "comp_wb_1" }),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "comp_tb_1"),
        new SpinUpForAutoShot(rc, "thoroughbred_2"),
        new Delay(rc, 15, true),
        new ShootAndWaitForced(rc),

        // new ReignChain(rc, "comp_wb_2", "comp_wb_3"),
        // new ReignChain(rc, "comp_wb_4", "comp_wb_5")
        new FollowPathFile(rc, "comp_tb_2"),
        new ChaseAndIntakeBroken(rc, true),
        new SpinUpForAutoShot(rc, "thoroughbred_3"),
        // new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_tb_3"),
        new Delay(rc, 15, true),
        new ShootAndWaitForced(rc),
        new FollowPathFile(rc, "comp_tb_4"),
        new ChaseAndIntakeBroken(rc, true),
        new SpinUpForAutoShot(rc, "thoroughbred_4"),
        // new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, "comp_tb_5"),
        new Delay(rc, 15, true),
        new ShootAndWaitForced(rc));
  }

  @Override
  public void initialize() {
    cmd.initialize();
  }
}
