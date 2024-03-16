// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

/** Add your docs here. */
public class MMField {
  static Pose2d leftBoundarySpeaker = new Pose2d(0, 5.5 - .356, new Rotation2d());
  static Pose2d rightBoundarySpeaker = new Pose2d(0, 5.5 + .356, new Rotation2d());
  public static double fieldX = 16.54;
  public static double fieldY = 8.23;
  private static Pose2d speakerTagPose = new Pose2d(4.64, 4.508, Rotation2d.fromDegrees(120));// x: fieldX/2 - 3.629, y:
                                                                                              // fieldY
  // + .393
  private static Pose2d nonSpeakerTagPose = new Pose2d(4.64, 3.722, Rotation2d.fromDegrees(-120));// x: fieldX/2 - 3.63
                                                                                                  // y:
  // fieldY/2-.392
  private static Pose2d fieldTagPose = new Pose2d(5.32, 4.115, Rotation2d.fromDegrees(0));// x:fieldx/2 -2.95, y:
                                                                                          // fieldy/2

  private static Transform2d distanceToStage = new Transform2d(1, 0, Rotation2d.fromDegrees(0));

  private static Pose2d blueSpeakerApproachPosition = speakerTagPose.plus(distanceToStage);

  private static Pose2d blueNonSpeakerApproachPosition = nonSpeakerTagPose.plus(distanceToStage);

  private static Pose2d blueFieldApproachPosition = fieldTagPose.plus(distanceToStage);

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

  private static Pose2d blueWooferHumanPlayerPose = new Pose2d(0.74, 6.68, Rotation2d.fromDegrees(-120));// -120

  private static Pose2d blueWooferNonHumanPlayerPose = new Pose2d(0.74, 4.29, Rotation2d.fromDegrees(120));// -120

  public static Pose2d getBlueWooferPose() {
    return blueWooferPose;
  }

  public static Pose2d getBlueWooferHumanPlayerPose() {
    return blueWooferHumanPlayerPose;
  }

  public static Pose2d getCurrentWooferHumanPlayerPose() {
    return getBluePose(blueWooferHumanPlayerPose);
  }

  public static Pose2d getBlueWooferNonHumanPlayerPose() {
    return blueWooferNonHumanPlayerPose;
  }

  public static Pose2d getCurrentWooferNonHumanPlayerPose() {
    return getBluePose(blueWooferNonHumanPlayerPose);
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

  public static Pose2d currentAmpPose() {
    return getBluePose(blueAmpPose);
  }

  public static Pose2d getBlueAmpPose() {
    return blueAmpPose;
  }

  // private static Pose2d blueStageFieldSide = new Pose2d(6.70, 4.0,
  // Rotation2d.fromDegrees(0));
  private static Pose2d blueStageFieldSide = blueFieldApproachPosition;

  public static Pose2d currentStagePose() {
    return getBluePose(blueStageFieldSide);
  }

  public static Pose2d getBlueStageFieldPose() {
    return blueStageFieldSide;
  }

  // private static Pose2d blueStageSpeakerSide = new Pose2d(3.056, 5.29,
  // Rotation2d.fromDegrees(210));
  // private static Pose2d blueStageSpeakerSide = new Pose2d(4.07, 5.63,
  // Rotation2d.fromDegrees(120));// 4.03, 5.59
  private static Pose2d blueStageSpeakerSide = blueSpeakerApproachPosition;

  public static Pose2d currentStageSpeakerSidePose() {
    return getBluePose(blueStageSpeakerSide);
  }

  public static Pose2d getBlueStageSpeakerSidePose() {
    return blueStageSpeakerSide;
  }

  // private static Pose2d blueStageNonSpeakerSide = new Pose2d(3.056, 2.924,
  // Rotation2d.fromDegrees(150));
  // private static Pose2d blueStageNonSpeakerSide = new Pose2d(4.09, 2.67,
  // Rotation2d.fromDegrees(-120));
  private static Pose2d blueStageNonSpeakerSide = blueNonSpeakerApproachPosition;

  public static Pose2d currentStageNonSpeakerPose() {
    return getBluePose(blueStageNonSpeakerSide);
  }

  public static Pose2d getBlueStageNonSpeakerSidePose() {
    return blueStageNonSpeakerSide;
  }

}
