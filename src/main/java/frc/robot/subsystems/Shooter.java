// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMTurnPIDController;
import frc.robot.MMUtilities.MMWaypoint;

public class Shooter extends SubsystemBase {
  RobotContainer rc;
  MMFiringSolution firingSolution = new MMFiringSolution(
      new MMWaypoint(1.3, -.111, 50),
      new MMWaypoint(2.5, -.078, 70),
      new MMWaypoint(3.97, -.057, 70));
  private Pose2d speakerPose;
  private MMTurnPIDController turnPidController = new MMTurnPIDController();
  private Pose2d currentPose;
  private double distanceToSpeaker;
  private double speakerTurnRate;

  private MMWaypoint desiredWaypoint;

  /** Creates a new Shooter. */
  public Shooter(RobotContainer rc) {
    this.rc = rc;
  }

  @Override
  public void periodic() {
    speakerPose = MMField.currentSpeakerPose();
    currentPose = rc.drivetrain.getState().Pose;

    Translation2d transformFromSpeaker = speakerPose.getTranslation().minus(currentPose.getTranslation());
    Rotation2d targetAngleSpeaker = transformFromSpeaker.getAngle();

    turnPidController.initialize(targetAngleSpeaker);

    speakerTurnRate = turnPidController.execute(currentPose.getRotation());
    distanceToSpeaker = transformFromSpeaker.getNorm();

    desiredWaypoint = firingSolution.calcSolution(distanceToSpeaker);

    SmartDashboard.putNumber("Velocity", desiredWaypoint.getVelocity());
    SmartDashboard.putNumber("Angle", desiredWaypoint.getAngle());
    SmartDashboard.putNumber("Distance To Target", distanceToSpeaker);
    SmartDashboard.putNumber("SpeakerTurnRate", speakerTurnRate);
    SmartDashboard.putString("Transform From target", transformFromSpeaker.toString());
    SmartDashboard.putNumber("Target Angle", targetAngleSpeaker.getDegrees());
  }

  public double getSpeakerTurnRate() {
    return speakerTurnRate;
  }

  public MMWaypoint getDesiredSpeakerWaypoint() {
    return desiredWaypoint;
  }

}
