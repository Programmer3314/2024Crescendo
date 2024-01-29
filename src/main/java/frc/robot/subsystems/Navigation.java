// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.RobotContainer;

public class Navigation extends SubsystemBase {
  RobotContainer rc;
  public static int visionUpdate = 0;
  private boolean hasLeftConeTarget;
  private double leftConeX;
  private double leftConeY;
  private LimelightTarget_Detector[] leftLimelightDetector;

  /** Creates a new Navigation. */
  public Navigation(RobotContainer rc) {
    this.rc = rc;
    SmartDashboard.putData("FieldX", rc.field);
  }

  @Override
  public void periodic() {
    if (true) {
      var lastResult = LimelightHelpers.getLatestResults("limelight-front").targetingResults;
      Pose2d pose = rc.drivetrain.getState().Pose;

      if (lastResult.valid && lastResult.targets_Fiducials.length > 1) {
        Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
        SmartDashboard.putString("llPose", llPose.toString());
        if (visionUpdate < 50 || pose.minus(llPose).getTranslation().getNorm() < 1) {
          rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
          visionUpdate++;
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
    leftLimelightDetector = LimelightHelpers.getLatestResults("limelight-left").targetingResults.targets_Detector;
    if (leftLimelightDetector.length > 0) {
      hasLeftConeTarget = true;
      leftConeX = leftLimelightDetector[0].tx_pixels;
      leftConeY = leftLimelightDetector[0].ty_pixels;
    }

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
}
