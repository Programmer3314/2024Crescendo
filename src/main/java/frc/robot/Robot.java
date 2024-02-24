// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Navigation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static DriverStation.Alliance alliance;

  // TODO: GLOBAL TODOs...
  // TODO: Control Documentation
  // TODO: Add Elevator Up/Down, Belt Up/Down to diagnostics (needs testing)
  // TODO: Tune up Elevator (speed up) (done)
  // TODO: Chase with Intake (controler button)
  // TODO: Shoot button to throw not shoot. (controller button)
  // TODO: Climb only (controller button)
  // TODO: Climb with Trap - on hold (controller button)
  // TODO: Clean up test autos vs. competition autos.
  // TODO: top 3 Autos for competition
  // TODO: Mustang Auto
  // TODO: Cleanup unused SHuffleboard output, remove unused fields from screen, title screens.
  // TODO: Repack Electronics cabinet, pack up and clear pneumatics from bottom draw and get rid of all useless stuff.
  // TODO: Spares
  // TODO: Run Belt In DropIntake


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
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.shooterSubsystem.stopMotors();

    // // Starts recording to data log
    DataLogManager.start();
    // // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    Navigation.visionUpdate = 0;

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
