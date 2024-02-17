// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.MMUtilities.MMController;
import frc.robot.MMUtilities.MMField;
import frc.robot.commands.Aim;
import frc.robot.commands.GoShoot;
import frc.robot.commands.GrabCone;
import frc.robot.commands.ShootTheConeOut;
import frc.robot.commands.Autos.AutoSamplerShootSmove;
import frc.robot.commands.Autos.FourNoteAuto;
import frc.robot.commands.Autos.MustangAuto;
import frc.robot.commands.Autos.StageSideAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Navigation;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  public final double MaxSpeed = 6; // 6 meters per second desired top speed
  public final double MaxAngularRate = Math.PI; // Half a rotation per second max angular velocity

  public final Field2d field = new Field2d();
  private Pose2d[] startPoseList = {
      new Pose2d(0.79, 6.71, Rotation2d.fromDegrees(-120)),
      new Pose2d(1.43, 5.5, Rotation2d.fromDegrees(180)),
      new Pose2d(0.79, 4.37, Rotation2d.fromDegrees(120))
  };
  private Pose2d[] notePoseList = {
      new Pose2d(6, 6.62, Rotation2d.fromDegrees(270)),
      new Pose2d(6.93, 6.44, Rotation2d.fromDegrees(270)),
      new Pose2d(7.6, 6.42, Rotation2d.fromDegrees(180)),
      new Pose2d(1.42, 6.33, Rotation2d.fromDegrees(270))
  };
  private Pose2d[] shootPoseList = {
      new Pose2d(2.4, 6.22, Rotation2d.fromDegrees(180)),
      MMField.getBlueWooferApproachPose(),
  };

  public MMController joystick = new MMController(0)
      .setDeadzone(.1 / 2)
      .setScaleXLeft(-MaxSpeed)
      .setScaleYLeft(-MaxSpeed)
      .setScaleXRight(-MaxAngularRate);
  public CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  Telemetry logger = new Telemetry(MaxSpeed);
  public Shooter shooterSubsystem = new Shooter(this);

  public Navigation navigation = new Navigation(this);

  private final SendableChooser<Command> autoChooser;
  public static SendableChooser<Pose2d> startPoseChooser;
  public static SendableChooser<Pose2d> noteChooser0;
  public static SendableChooser<Pose2d> shootChooser0;
  public static SendableChooser<Pose2d> noteChooser1;
  public static SendableChooser<Pose2d> shootChooser1;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(joystick.getLeftYSmoothed())
            .withVelocityY(joystick.getLeftXSmoothed())
            .withRotationalRate(joystick.getRightXSmoothed())));

    // TODO: review the following two controls...
    // This is why the buttons need to be held. I believe the idea was to allow the driver to change their mind.             
    joystick.rightBumper().onTrue(new InstantCommand(() -> shooterSubsystem.setIntakeFlag(true)))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setIntakeFlag(false)));

    joystick.rightTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setShootFlag(true)))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setShootFlag(false)));

        // joystick.b().onTrue(new
    // InstantCommand(()->shooterSubsystem.setReverseIntakeFlag(true)));
    joystick.a().whileTrue(new Aim(this));
    // joystick.x().onTrue(new
    // InstantCommand(()->shooterSubsystem.setRunDiagnostic(true)));
    // joystick.y().whileTrue(new ChaseCone(this));
    joystick.leftTrigger().onTrue(new GoShoot(this));
    joystick.leftBumper().onTrue(
        new ParallelCommandGroup(drivetrain.runOnce(() -> drivetrain.seedFieldRelative())));
    joystick.button(8).onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative(MMField.currentWooferPose())));
    joystick.button(7).onTrue(new InstantCommand(() -> shooterSubsystem.setRunDiagnostic(true)));
    joystick.b().onTrue(new InstantCommand(() -> shooterSubsystem.setIntakeDown()))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setIntakeUp()));
    joystick.x().onTrue(new InstantCommand(() -> shooterSubsystem.setElevatorVoltage(0)));
    joystick.y().onTrue(new InstantCommand(() -> shooterSubsystem.setElevatorVoltage(6)))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setElevatorVoltage(0)));

    // Set<Subsystem> set = new HashSet<Subsystem>();
    // set.add(drivetrain);
    // joystick.x().onTrue(new DeferredCommand(() -> runDeferredTest(), set));
    // SmartDashboard.putData("Run Deferred Test",
    // new DeferredCommand(() -> runDeferredTest(), set));

    // joystick.x().whileTrue(new PathFindTo(this,
    // MMField::getBlueWooferApproachPose));
    // joystick.y().whileTrue(new AutoSamplerShootSmove(this));
    // joystick.a().whileTrue(new NotAim(this));

    // joystick.y().whileTrue(new InstantCommand(
    // () -> claw.armExtensionRot(30)));
    // joystick.x().whileTrue(new InstantCommand(
    // () -> claw.armExtensionRot(0)));

    // joystick.a().whileTrue(new InstantCommand(() -> claw.armRotationRot(0)));
    // joystick.b().whileTrue(new InstantCommand(() -> claw.armRotationRot(-0.2)));
    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    // .applyRequest(() -> point.withModuleDirection(new
    // Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press

    // joystick.rightBumper().onTrue(new InstantCommand(
    // () -> claw.openClaw()));
    // joystick.rightTrigger().onTrue(new InstantCommand(
    // () -> claw.closeClaw()));
    // joystick.leftTrigger().whileTrue(new ChaseCone(this));

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
    // Default auto will be `Commands.none()`
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("none", Commands.none());
    autoChooser.addOption("ShootSmove", new AutoSamplerShootSmove(this));
    autoChooser.addOption("MustangAuto", new MustangAuto(this));
    autoChooser.addOption("StageSideAuto", new StageSideAuto(this));
    autoChooser.addOption("FourNoteAuto", new FourNoteAuto(this));
    autoChooser.setDefaultOption("none", Commands.none());
    SmartDashboard.putData("Auto Mode", autoChooser);

    // noteChooser0 = new SendableChooser<Pose2d>();
    // noteChooser1 = new SendableChooser<Pose2d>();
    // shootChooser0 = new SendableChooser<Pose2d>();
    // shootChooser1 = new SendableChooser<Pose2d>();

    noteChooser0 = fillNoteChooser("Note 1");
    noteChooser1 = fillNoteChooser("Note 2");

    shootChooser0 = fillShootPoseChooser("Shoot Pose 1");
    shootChooser1 = fillShootPoseChooser("Shoot Pose 2");

    startPoseChooser = fillStartPoseChooser("Pick Start Pose");
  }

  private SendableChooser<Pose2d> fillShootPoseChooser(String shootPoseName) {
    SendableChooser<Pose2d> shootChooser = new SendableChooser<Pose2d>();
    for (int i = 0; i < shootPoseList.length; i++) {
      shootChooser.addOption("Shoot: " + i, shootPoseList[i]);
    }
    SmartDashboard.putData(shootPoseName, shootChooser);
    return shootChooser;
  }

  private SendableChooser<Pose2d> fillStartPoseChooser(String startPoseName) {
    SendableChooser<Pose2d> startChooser = new SendableChooser<Pose2d>();
    for (int i = 0; i < startPoseList.length; i++) {
      startChooser.addOption("Start: " + i, startPoseList[i]);
    }
    SmartDashboard.putData(startPoseName, startChooser);
    return startChooser;
  }

  private SendableChooser<Pose2d> fillNoteChooser(String noteChooserName) {
    SendableChooser<Pose2d> noteChooser = new SendableChooser<Pose2d>();
    for (int i = 0; i < notePoseList.length; i++) {
      noteChooser.addOption("Note: " + i, notePoseList[i]);
    }
    SmartDashboard.putData(noteChooserName, noteChooser);
    return noteChooser;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  // strip this down to use none of our code...
  // hard code the poses, etc.
  // Michael Jansen says it is our code. Let's find out.
  public Command runDeferredTest() {
    PathConstraints trajectoryConstraints = new PathConstraints(1.5, 3, 2 * Math.PI, 4 * Math.PI);
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(new Translation2d(1.8, 5.5), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.6, 5.5), Rotation2d.fromDegrees(180)),
        new Pose2d(new Translation2d(1.4, 5.5), Rotation2d.fromDegrees(180)));
    PathPlannerPath path = new PathPlannerPath(bezierPoints,
        trajectoryConstraints,
        new GoalEndState(0, Rotation2d.fromDegrees(180)));
    path.preventFlipping = false;
    return AutoBuilder.followPath(path);
  }
}