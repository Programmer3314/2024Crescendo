// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class Navigation extends SubsystemBase {
  RobotContainer rc;
  public static int visionUpdate = 0;
  private boolean hasLeftConeTarget;
  private double leftConeX;
  private double leftConeY;
  private LimelightTarget_Detector[] leftLimelightDetector;
  private String limelightFrontName = "limelight-front";
  private String limelightBackUpName = "limelight-backup";

  private double llHeartBeat = 0;
  // private ArrayList<Double> xVelocity = new ArrayList<Double>();
  // private ArrayList<Double> yVelocity = new ArrayList<Double>();
  // private ArrayList<Double> angVelocity = new ArrayList<Double>();
  public static Pose2d predictedPose;
  // private int predictionCycles;
  // double timeToShoot = .1;
  private Pose2d currentPose;

  /** Creates a new Navigation. */
  public Navigation(RobotContainer rc) {
    this.rc = rc;
    SmartDashboard.putData("FieldX", rc.field);
    LimelightHelpers.setPipelineIndex("limelight-bd", 0);
    LimelightHelpers.setPipelineIndex("limelight-backup", 0);
  }

  @Override
  public void periodic() {
    Pose2d pose = rc.drivetrain.getState().Pose;
    currentPose = pose;

    if (true) {
      var lastResult = LimelightHelpers.getLatestResults(limelightBackUpName).targetingResults;
      SmartDashboard.putNumber("LL Heartbeat", lastResult.timestamp_LIMELIGHT_publish);

      if (lastResult.timestamp_LIMELIGHT_publish != llHeartBeat) {
        llHeartBeat = lastResult.timestamp_LIMELIGHT_publish;

        if (lastResult.valid && lastResult.targets_Fiducials.length > 0) {
          Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
          SmartDashboard.putString("llPose", llPose.toString());
          double margin = pose.minus(llPose).getTranslation().getNorm();
          if (visionUpdate < 50
              || margin < .25
              || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
            rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            visionUpdate++;
          }
        }
      }

      rc.field.setRobotPose(pose);
      SmartDashboard.putString("MMpose", pose.toString());
    }

    if (true) {
      var lastResult = LimelightHelpers.getLatestResults(limelightFrontName).targetingResults;
      SmartDashboard.putNumber("LL Heartbeat", lastResult.timestamp_LIMELIGHT_publish);

      if (lastResult.timestamp_LIMELIGHT_publish != llHeartBeat) {
        llHeartBeat = lastResult.timestamp_LIMELIGHT_publish;

        if (lastResult.valid && lastResult.targets_Fiducials.length > 0) {
          Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
          SmartDashboard.putString("llPose", llPose.toString());
          double margin = pose.minus(llPose).getTranslation().getNorm();
          if (visionUpdate < 50
              || margin < .25
              || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
            rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            visionUpdate++;
          }
        }
      }

      rc.field.setRobotPose(pose);
      SmartDashboard.putString("MMpose", pose.toString());
    }

    // var llpython = LimelightHelpers.getPythonScriptData("limelight-left");
    // hasLeftConeTarget = llpython[0] > .5;
    // leftConeX = llpython[1];
    // leftConeY = llpython[2];
    hasLeftConeTarget = false;
    // leftLimelightDetector =
    // LimelightHelpers.getLatestResults("limelight-left").targetingResults.targets_Detector;
    // if (leftLimelightDetector.length > 0) {
    // hasLeftConeTarget = true;
    // leftConeX = leftLimelightDetector[0].tx_pixels;
    // leftConeY = leftLimelightDetector[0].ty_pixels;
    // }

    SmartDashboard.putNumber("TX:", leftConeX);
    SmartDashboard.putNumber("TY", leftConeY);
  }

  public double getLeftConeX() {
    return leftConeX;
  }

  public double getLeftConeY() {
    return leftConeY;
  }

  public boolean hasLeftConeTarget() {
    return hasLeftConeTarget;
  }

  public String autoLeftOrRight() {
    if (leftConeX > 300) {
      return "NoteRight";
    } else {
      return "NoteStraight";
    }
  }

  public double getDistanceToSpeaker() {
    Pose2d currentPose = rc.drivetrain.getState().Pose;
    Translation2d target = MMField.currentSpeakerPose().getTranslation();
    double distance = currentPose.getTranslation().minus(target).getNorm();
    return distance;
  }

  // public double getPredictedDistanceToSpeaker() {
  // Pose2d currentPose = predictedPose;
  // Translation2d target = MMField.currentSpeakerPose().getTranslation();
  // double distance = currentPose.getTranslation().minus(target).getNorm();
  // return distance;
  // }

  // public void updatePredictedPosition(double x, double y, double r) {
  // xVelocity.add(x);
  // yVelocity.add(y);
  // angVelocity.add(r);
  // limitArrayLists();
  // syncPredictedPosition();
  // }

  // public void syncPredictedPosition() {
  // double xVelocitySum = 0;
  // double yVelocitySum = 0;
  // double angVelocitySum = 0;
  // for (int i = 0; i < xVelocity.size(); i++) {
  // xVelocitySum += xVelocity.get(i);
  // }
  // for (int i = 0; i < yVelocity.size(); i++) {
  // yVelocitySum += yVelocity.get(i);
  // }
  // for (int i = 0; i < angVelocity.size(); i++) {
  // angVelocitySum += angVelocity.get(i);
  // }
  // double averageX = xVelocitySum / xVelocity.size();
  // double averageY = yVelocitySum / yVelocity.size();
  // double averageAng = angVelocitySum / angVelocity.size();

  // predictedPose = currentPose.plus(new Transform2d())
  // }

  // public void limitArrayLists() {
  // if (xVelocity.size() > predictionCycles) {
  // xVelocity.remove(xVelocity.size() - 1);
  // }
  // if (yVelocity.size() > predictionCycles) {
  // yVelocity.remove(yVelocity.size() - 1);
  // }
  // if (angVelocity.size() > predictionCycles) {
  // angVelocity.remove(angVelocity.size() - 1);
  // }
  // }
}
