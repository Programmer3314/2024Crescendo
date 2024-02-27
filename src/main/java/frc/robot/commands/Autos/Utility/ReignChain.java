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
public class ReignChain extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;
  String goToShot;
  String goToNote;

  String[] goToShots;
  String[] goToNotes;

  /** Creates a new TwoShotAuto. */
  public ReignChain(RobotContainer rc, String goToNote, String goToShot) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.goToNote = goToNote;
    this.goToShot = goToShot;
  }

  public ReignChain(RobotContainer rc, String[] goToNotes, String[] goToShots) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.goToNotes = goToNotes;
    this.goToShots = goToShots;
  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new InstantCommand(() -> rc.shooterSubsystem.setIntakeFlag(true)),
        new FollowPathFile(rc, goToNote),
        new InstantCommand(() -> rc.shooterSubsystem.setAimFlag(true)),
        new FollowPathFile(rc, goToShot),
        new ShootAndWait(rc));
    cmd.initialize();
  }
}
