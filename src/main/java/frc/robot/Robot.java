// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Navigation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static DriverStation.Alliance alliance;
  public static int resetDriverValue = 1;
  public static Rotation2d allianceSpeakerRotation = new Rotation2d();

  // TODO: GLOBAL TODOs...
  // TODO: Try to reduce can errors by a) reduce odometry Freq using
  // alternate drivetrain constructor with freq parameter (maybe 200?,150?)
  // or b) splitting the can bus
  // TODO: Control Documentation
  // TODO: Climb with Trap (controller button) - Add AbortClimb
  // TODO: Migrate Lessons learned from Warmblood to Arabian
  // TODO: Mustang Auto - extend to 4 pieces at least and add shoot in place,
  // clear
  // TODO: Catch Dual note intake, and reverse
  // TODO: intake failure make sure that if a state machine reset is need that it
  // is automatic.
  // TODO: Cleanup unused Shuffleboard output, remove unused fields from screen,
  // title screens.
  // TODO: Repack Electronics cabinet, pack up and clear pneumatics from bottom
  // draw and get rid of all useless stuff.
  // TODO: Spares
  // TODO: add manual dpad up/down to adjust the shooter margin on the fly for the
  // operator
  // TODO: Work on LEDs

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();

    // Shuffleboard.getTab("Field").addString("pose", () ->
    // m_robotContainer.drivetrain.getState().Pose.toString())
    // .withWidget(BuiltInWidgets.kField);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    // Merged Reset Driver value from below
    // Captured alliaceSpeakerRotation (0 or 180 degrees)
    if (alliance == null) {
      var allianceAttempt = DriverStation.getAlliance();
      if (allianceAttempt.isPresent()) {
        alliance = allianceAttempt.get();
        if (alliance.equals(Alliance.Red)) {
          resetDriverValue = -1;
          allianceSpeakerRotation = Rotation2d.fromDegrees(0);
        } else {
          resetDriverValue = 1;
          allianceSpeakerRotation = Rotation2d.fromDegrees(180);
        }
        SmartDashboard.putString("alliance", alliance.toString());
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    m_robotContainer.shooterSubsystem.stopMotors();
    m_robotContainer.shooterSubsystem.resetStateMachine();
    Navigation.visionUpdate = 0;

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
    // Moved to disabled periodic
    // if (alliance != null) {
    // resetDriverValue = alliance.equals(Alliance.Red) ? -1 : 1;
    // }
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.shooterSubsystem.stopMotors();

    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    Navigation.visionUpdate = 0;

  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    // DataLogManager.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.shooterSubsystem.stopMotors();

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

  public static void resetVisionUpdate() {
    Navigation.visionUpdate = 0;
  }

}
