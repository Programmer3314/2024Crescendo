// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/** Add your docs here. */
public class MMField {
    public static Translation2d getBlueTranslation(Translation2d translation) {
    if (Robot.alliance.equals(DriverStation.Alliance.Red)) {
      return GeometryUtil.flipFieldPosition(translation);
    }
    return translation;
  }

  public static Rotation2d getBlueRotation(Rotation2d rotation) {
    if (Robot.alliance.equals(DriverStation.Alliance.Red)) {
      return GeometryUtil.flipFieldRotation(rotation);
    }
    return rotation;
  }

  public static Pose2d getBluePose(Pose2d pose) {
    if (Robot.alliance.equals(DriverStation.Alliance.Red)) {
      return GeometryUtil.flipFieldPose(pose);
    }
    return pose;
  }
}
