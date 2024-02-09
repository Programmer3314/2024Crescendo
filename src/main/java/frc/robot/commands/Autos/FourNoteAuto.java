// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.commands.Aim;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.PathFindTo;
import frc.robot.commands.ShootAndWait;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteAuto extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;
  /** Creates a new FourNoteAuto. */
  public FourNoteAuto(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
  @Override
  public void initialize() {
    rc.drivetrain.seedFieldRelative(RobotContainer.startPoseChooser.getSelected());
    cmd = new SequentialCommandGroup();
    cmd.addCommands(new ShootAndWait(rc),
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    new FollowPathFile(rc, "Note1"),
    new Aim(rc),
    new AutoShoot(rc), 
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    new FollowPathFile(rc, "Note2"),
    new Aim(rc), 
    new AutoShoot(rc),
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    new FollowPathFile(rc, "Note3"),
     new Aim(rc),
     new AutoShoot(rc));
  }
}
