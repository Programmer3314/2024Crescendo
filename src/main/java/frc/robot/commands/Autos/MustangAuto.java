// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMDeferredCommand;
import frc.robot.MMUtilities.MMField;

public class MustangAuto extends MMDeferredCommand<SequentialCommandGroup> {

  public MustangAuto(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    cmd.addCommands(new StandardAutoInit(rc, MMField.currentWooferPose())
        .setPipeLine(0, 1));
    if (RobotContainer.noteChooser0.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser0.getSelected(),
          RobotContainer.shootChooser0.getSelected()));
    }
    if (RobotContainer.noteChooser1.getSelected() != null) {
      cmd.addCommands(new MustangAutoCycle(rc, RobotContainer.noteChooser1.getSelected(),
          RobotContainer.shootChooser1.getSelected()));
    }
    cmd.initialize();
  }
}
