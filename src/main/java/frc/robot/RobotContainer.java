// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
// import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.MMUtilities.MMController;
import frc.robot.commands.GoShoot;
import frc.robot.commands.GrabCone;
import frc.robot.commands.ShootTheConeOut;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  final double MaxSpeed = 6; // 6 meters per second desired top speed
  final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  MMController joystick = new MMController(0, .1 / 2); // My joystick
  public CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  // TODO: go back to static Claw
  public Claw claw = new Claw(this);
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  // .withIsOpenLoop(true);
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(-joystick.getLeftYSmoothed() * MaxSpeed)
            .withVelocityY(-joystick.getLeftXSmoothed() * MaxSpeed)
            .withRotationalRate(-joystick.getRightXSmoothed() * MaxAngularRate)));

    // joystick.y().whileTrue(new ShootTheConeOut(this));
    // joystick.x().whileTrue(new GrabCone(this));
    joystick.b().whileTrue(new GoShoot(this));

    // joystick.y().whileTrue(new InstantCommand(
    //     () -> claw.armExtensionRot(30)));
    // joystick.x().whileTrue(new InstantCommand(
    //     () -> claw.armExtensionRot(0)));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
        // .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    joystick.rightBumper().onTrue(new InstantCommand(
        () -> claw.openClaw()));
    joystick.rightTrigger().onTrue(new InstantCommand(
        () -> claw.closeClaw()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {

    configureBindings();

    //
    // PathPlanner
    //
    // Register named commands
    NamedCommands.registerCommand("dropCone", new ShootTheConeOut(this));
    NamedCommands.registerCommand("grabCone", new GrabCone(this));
    // Set Up Autochooser
    // TODO: review AutoBuilder to put all autos in dropdown


    // Default auto will be `Commands.none()`
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");

    //
    // PathPlanner
    //
    // return new PathPlannerAuto("2024TestAuto01");
    // return new PathPlannerAuto("TestAuto");
    // ALternate from chooser
    return autoChooser.getSelected();

  }
}