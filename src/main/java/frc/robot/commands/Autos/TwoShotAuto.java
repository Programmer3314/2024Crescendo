// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.ShootAndWait;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoShotAuto extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public TwoShotAuto(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 1),
        // turn on shoot flag
        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),

        // turn on intake flag
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        // drive back 1.09 meters
        // new DriveForwardDist(rc, 1.09, -.5),
        new FollowPathFile(rc, "BackUp"),
        // shoot then intake
        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "TwoBall-2"),
        new ShootAndWait(rc),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, "TwoBall-3"),
        new ShootAndWait(rc)

    );
    cmd.initialize();
  }
}
