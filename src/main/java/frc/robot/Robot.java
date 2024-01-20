// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final Field2d field = new Field2d();

  public static int visionUpdate = 0;

  public static DriverStation.Alliance alliance;

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    SmartDashboard.putData("FieldX", field);
    // Shuffleboard.getTab("Field").addString("pose", () ->
    // m_robotContainer.drivetrain.getState().Pose.toString())
    // .withWidget(BuiltInWidgets.kField);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    if (true) {
      var lastResult = LimelightHelpers.getLatestResults("limelight-right").targetingResults;
      Pose2d pose = m_robotContainer.drivetrain.getState().Pose;

      if (lastResult.valid && lastResult.targets_Fiducials.length > 0) {
        Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
        SmartDashboard.putString("llPose", llPose.toString());
        if (visionUpdate < 50 || pose.minus(llPose).getTranslation().getNorm() < 1) {
          m_robotContainer.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
          visionUpdate++;
        }
      }

      field.setRobotPose(pose);
      SmartDashboard.putString("MMpose", pose.toString());

    }

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (alliance == null) {
      var allianceAttempt = DriverStation.getAlliance();
      if (allianceAttempt.isPresent()) {
        alliance = allianceAttempt.get();
        SmartDashboard.putString("alliance", alliance.toString());
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    visionUpdate = 0;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // // Starts recording to data log
    DataLogManager.start();
    // // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    visionUpdate = 0;

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    DataLogManager.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}
