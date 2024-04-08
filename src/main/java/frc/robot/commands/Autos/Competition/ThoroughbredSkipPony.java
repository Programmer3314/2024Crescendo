// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos.Competition;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.ChaseAndIntakeBroken;
import frc.robot.commands.ChaseNoteBroken;
import frc.robot.commands.Delay;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.DriveForwardDistBroken;
import frc.robot.commands.DriveToNote;
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
public class ThoroughbredSkipPony extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public ThoroughbredSkipPony(RobotContainer rc) {
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
        new FollowPathFile(rc, "comp_tbs_1"),
        new DriveToNote(rc),
        new DriveForwardDist(rc, 2, -2), // maybe extend l8r
        new FollowPathFile(rc, "comp_tbsp_2"),
        new ChaseAndIntakeBroken(rc, true),
        // new SpinUpForAutoShot(rc, "thoroughbred_skip_3"),
        // new FollowPathFile(rc, "comp_tbs_3"),
        // new Delay(rc, 5, true),
        // new ShootAndWaitForced(rc),
        // new FollowPathFile(rc, "comp_tbs_4"),
        new SpinUpForAutoShot(rc, "thoroughbred_skip_3"),
        new FollowPathFile(rc, "comp_tbsp_3"),
        new Delay(rc, 5, true),
        new ShootAndWaitForced(rc),
        new FollowPathFile(rc, "comp_tbsp_4"),
        new ChaseAndIntakeBroken(rc, true),
        new SpinUpForAutoShot(rc, "thoroughbred_skip_pony_4"),
        new FollowPathFile(rc, "comp_tbsp_5"),
        new Delay(rc, 15, true),
        new ShootAndWaitForced(rc),
        new FollowPathFile(rc, "comp_tbsp_6"),
        new ChaseAndIntakeBroken(rc, true));
  }

  @Override
  public void initialize() {
    cmd.initialize();
  }
}
