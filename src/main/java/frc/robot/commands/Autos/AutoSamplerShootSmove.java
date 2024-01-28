// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.ChaseCone;
import frc.robot.commands.DriveForwardDist;
import frc.robot.commands.FollowPathFile;
import frc.robot.commands.NotAim;
import frc.robot.commands.PathFindTo;
import frc.robot.subsystems.Navigation;

// TODO: review options for handling sequential commands
// Could be handled in the binding itself, either with a Commands.sequential... or as Noted below.
// Or like this...

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoSamplerShootSmove extends SequentialCommandGroup {
  /** Creates a new ShootSmove. */
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
        .setPipeLine(0, 1),
        new FollowPathFile(rc, "Exit"), 
       // new FollowPathFile(rc, rc.navigation::autoLeftOrRight), 
       new ChaseCone(rc),
       new DriveForwardDist(rc, 0.61, .5),
        new PathFindTo(rc, MMField::getBlueWooferApproachPose)
        .setRotationDelayDistance(0.2),
        new NotAim(rc),
        new DriveForwardDist(rc, .2, .5));

  }
}
