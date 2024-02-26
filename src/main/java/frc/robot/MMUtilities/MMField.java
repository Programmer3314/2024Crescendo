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
 static Pose2d leftBoundarySpeaker = new Pose2d(0,5.5-.356,new Rotation2d());
 static Pose2d rightBoundarySpeaker = new Pose2d(0,5.5+.356, new Rotation2d());
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

  public static Pose2d blueSpeakerPose = new Pose2d(new Translation2d(0, 5.55), new Rotation2d());

  public static Pose2d currentSpeakerPose() {
    // the following line flips if red
    return getBluePose(blueSpeakerPose);
  }

  private static Pose2d blueWooferPose = new Pose2d(1.4, 5.5, Rotation2d.fromDegrees(180));

  public static Pose2d getBlueWooferPose() {
    return blueWooferPose;
  }

  public static Pose2d currentWooferPose() {
    return getBluePose(blueWooferPose);
  }
  public static Pose2d currentLeftBoundaryPose() {

    return getBluePose(leftBoundarySpeaker);
  }
  public static Pose2d currentRightBoundaryPose() {

    return getBluePose(rightBoundarySpeaker);
  }

  private static Pose2d blueWooferApproachPose = new Pose2d(1.6, 5.5, Rotation2d.fromDegrees(180));

  public static Pose2d getBlueWooferApproachPose() {
    return blueWooferApproachPose;
  }

  public static Pose2d currentWooferApproachPose() {
    return getBluePose(blueWooferApproachPose);
  }

  private static Pose2d blueAmpPose = new Pose2d(1.90, 7.65, Rotation2d.fromDegrees(-90));

  public static Pose2d currentAmpPose(){
    return getBluePose(blueAmpPose);
  }

  public static Pose2d getBlueAmpPose(){
    return blueAmpPose;
  }

  private static Pose2d blueStageFieldSide = new Pose2d(6.692, 4.11-.18, Rotation2d.fromDegrees(0));

  public static Pose2d currentStagePose(){
    return getBluePose(blueStageFieldSide);
  }

  public static Pose2d getBlueStageFieldPose(){
    return blueStageFieldSide;
  }

   private static Pose2d blueStageLeftSide = new Pose2d(3.056, 5.29, Rotation2d.fromDegrees(210));

  public static Pose2d currentStageLeftPose(){
    return getBluePose(blueStageLeftSide);
  }

  public static Pose2d getBlueStageLeftPose(){
    return blueStageLeftSide;
  }

  private static Pose2d blueStageRightSide = new Pose2d(3.056, 2.924, Rotation2d.fromDegrees(150));

  public static Pose2d currentStageRightPose(){
    return getBluePose(blueStageRightSide);
  }

  public static Pose2d getBlueStageRightPose(){
    return blueStageRightSide;
  }

}
