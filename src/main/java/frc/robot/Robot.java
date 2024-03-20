// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.MMUtilities.MMWaypoint;
import frc.robot.commands.Autos.Competition.Arabian;
import frc.robot.commands.Autos.Competition.CrazyHorse;
import frc.robot.commands.Autos.Competition.HorseShoe;
import frc.robot.commands.Autos.Competition.HorseShoeTwo;
import frc.robot.commands.Autos.Competition.Pony;
import frc.robot.commands.Autos.Competition.Thoroughbred;
import frc.robot.subsystems.Navigation;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  public static DriverStation.Alliance alliance;
  public static int resetDriverValue = 1;
  public static Rotation2d allianceSpeakerRotation = new Rotation2d();

  // TODO: GLOBAL TODOs...
  // TODO: Try Vision Threading and latency calcs

  // TODO: Before Weekend - get autos in place (roughed out)
  // TODO: Get HS2 in place with attempt at 5th Note
  // TODO: Split the can bus
  // TODO: Control Documentation
  // TODO: Climb with Trap (controller button) - Add AbortClimb
  // TODO: Investigate raising drivetrain current limitting to 80 (or above)
  // TODO: Mustang Auto - extend to 4 pieces at least and add shoot in place,
  // clear
  // TODO: Catch Dual note intake, and reverse
  // TODO: intake failure make sure that if a state machine reset is need that it
  // is automatic.
  // TODO: Cleanup unused Shuffleboard output, remove unused fields from screen,
  // title screens.
  // TODO: Extended Diagnostics

  @Override
  public void robotInit() {

    m_robotContainer = new RobotContainer();
    m_robotContainer.climber.setClimbFlag(false);
    m_robotContainer.climber.setClimbUnwindFlag(false);
    pdh.setSwitchableChannel(true);
    SmartDashboard.putBoolean("AutosConfigured", false);
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
    m_robotContainer.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
    m_robotContainer.oppController.getHID().setRumble(RumbleType.kBothRumble, 0);
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

        // TODO: Build Autos Here...
        m_robotContainer.pony = new Pony(m_robotContainer);
        m_robotContainer.arabian = new Arabian(m_robotContainer);
        m_robotContainer.crazyHorse = new CrazyHorse(m_robotContainer);
        m_robotContainer.horseshoe = new HorseShoe(m_robotContainer);
        m_robotContainer.horseShoeTwo = new HorseShoeTwo(m_robotContainer);
        m_robotContainer.thoroughbred = new Thoroughbred(m_robotContainer);
        SmartDashboard.putBoolean("AutosConfigured", true);
        SmartDashboard.putString("alliance", alliance.toString());
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    m_robotContainer.climber.setClimbFlag(false);
    m_robotContainer.climber.setClimbUnwindFlag(false);
    m_robotContainer.climber.setClimbPos();

    m_robotContainer.shooterSubsystem.stopMotors();
    m_robotContainer.shooterSubsystem.resetStateMachine();
    Navigation.visionUpdate = 0;

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.navigation.setAutoVisionOn(false);

    // pdh.setSwitchableChannel(true);

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

    m_robotContainer.climber.resetStateMachine();
    m_robotContainer.shooterSubsystem.resetStateMachine();
    m_robotContainer.shooterSubsystem.setReadyToAutoShoot(false);
    m_robotContainer.climber.setClimbFlag(false);
    m_robotContainer.climber.setClimbUnwindFlag(false);
    m_robotContainer.climber.setClimbPos();
    m_robotContainer.shooterSubsystem.setAutoForce(false);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.shooterSubsystem.stopMotors();

    // Starts recording to data log
    DataLogManager.start();
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
    m_robotContainer.navigation.setAutoVisionOn(true);
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
    Pose2d testPosition = new Pose2d(7.14, 3.14, Rotation2d.fromDegrees(171.38));
    MMWaypoint waypoint = m_robotContainer.shooterSubsystem.calcManualFiringSolution(testPosition);
    System.out.println("Angle: " + waypoint.getAngle());
    System.out.println("Left: " + waypoint.getLeftVelocity());
    System.out.println("Right: " + waypoint.getRightVelocity());
    System.out.println("Velocity: " + waypoint.getVelocity());

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
