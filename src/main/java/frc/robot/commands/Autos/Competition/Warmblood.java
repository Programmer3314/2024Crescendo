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
import frc.robot.commands.Autos.Utility.Reign;
import frc.robot.commands.Autos.Utility.ReignChain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Warmblood extends MMDeferredCommand<SequentialCommandGroup> {
  RobotContainer rc;

  /** Creates a new TwoShotAuto. */
  public Warmblood(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

  }

  @Override
  public void initialize() {
    cmd = new SequentialCommandGroup();
    // TODO: HIGH PRIORITY Fix Pipelines
    cmd.addCommands(
        new StandardAutoInit(rc, MMField.currentWooferPose())
            .setPipeLine(0, 0, 0),
        new ShootAndWait(rc),
        new Reign(rc, new String[] { "comp_wb_1" }),
        new ReignChain(rc, "comp_wb_2", "comp_wb_3"),
        new ReignChain(rc, "comp_wb_4", "comp_wb_5")

    );
    cmd.initialize();
  }
}
