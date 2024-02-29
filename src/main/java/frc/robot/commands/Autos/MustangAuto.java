// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.DriveForwardDist;

public class MustangAuto extends MMDeferredCommand<SequentialCommandGroup> {

  public MustangAuto(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    cmd.addCommands(new StandardAutoInit(rc, MMField.currentWooferPose())
        .setPipeLine(0, 0, 0));
    cmd.addCommands(new DriveForwardDist(rc, 0.34, -0.5));
    if (RobotContainer.noteChooser0.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser0.getSelected(),
          RobotContainer.shootChooser0.getSelected()));
    }
    if (RobotContainer.noteChooser1.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser1.getSelected(),
          RobotContainer.shootChooser1.getSelected()));
    }
    if (RobotContainer.noteChooser2.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser2.getSelected(),
          RobotContainer.shootChooser2.getSelected()));
    }
    if (RobotContainer.noteChooser3.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser3.getSelected(),
          RobotContainer.shootChooser3.getSelected()));
    }
    cmd.initialize();
  }
}
