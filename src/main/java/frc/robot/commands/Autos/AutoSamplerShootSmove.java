// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.ChaseNote;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.NotAim;
import frc.robot.commands.PathFindTo;

public class AutoSamplerShootSmove extends SequentialCommandGroup {
  RobotContainer rc;

  public AutoSamplerShootSmove(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
    addCommands(
        // new FollowPathFile(rc,"Exit"),
        // new ChaseCone(rc),
        // new FindShoot(rc),
        // new NotAim(rc));
        new StandardAutoInit(rc, MMField.getBlueWooferPose())
            .setPipeLine(0, 0, 0),
        new FollowPathFile(rc, "Exit"),
        // new FollowPathFile(rc, rc.navigation::autoLeftOrRight),
        new ChaseNote(rc),
        new DriveForwardDist(rc, 0.41, 1)
            .setRobotStop(false),
        new DriveForwardDist(rc, 0.2, .5),
        new PathFindTo(rc, MMField::getBlueWooferApproachPose)
            .setRotationDelayDistance(0.2),
        new NotAim(rc),
        new DriveForwardDist(rc, .2, .5));

  }
}
