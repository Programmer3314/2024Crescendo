// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;

public class badAuto extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  public badAuto(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
  }
  @Override
  public void initialize() {
    // rc.drivetrain.seedFieldRelative(RobotContainer.startPoseChooser.getSelected());
    // cmd = new SequentialCommandGroup();
    // cmd.addCommands(new ShootAndWait(rc),
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    // new FollowPathFile(rc, "Note1"),
    // new Aim(rc),
    // new AutoShoot(rc), 
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    // new FollowPathFile(rc, "Note2"),
    // new Aim(rc), 
    // new AutoShoot(rc),
    // new InstantCommand(rc::shooterSubsystem.setIntakeFlag(true)),
    // new FollowPathFile(rc, "Note3"),
    //  new Aim(rc),
    //  new AutoShoot(rc));
  }
}
