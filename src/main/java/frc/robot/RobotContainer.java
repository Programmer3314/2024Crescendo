// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.MMUtilities.MMController;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.commands.Aim;
import frc.robot.commands.AimToWall;
import frc.robot.commands.ChaseAndIntake;
import frc.robot.commands.ChaseAndIntakeBroken;
import frc.robot.commands.ClawsUpAndIndex;
import frc.robot.commands.FullChuckLow;
import frc.robot.commands.FullClimb;
import frc.robot.commands.GoAmp;
import frc.robot.commands.GoClimb;
import frc.robot.commands.GoShoot;
import frc.robot.commands.SetColor;
import frc.robot.commands.ShootTheConeOut;
import frc.robot.commands.SignalForNote;
import frc.robot.commands.StartClimb;
import frc.robot.commands.StartClimbManual;
import frc.robot.commands.Autos.AutoSamplerShootSmove;
import frc.robot.commands.Autos.MustangAuto;
import frc.robot.commands.Autos.StageSideAuto;
import frc.robot.commands.Autos.Competition.Arabian;
import frc.robot.commands.Autos.Competition.CrazyHorse;
import frc.robot.commands.Autos.Competition.HorseShoe;
import frc.robot.commands.Autos.Competition.HorseShoeTwo;
import frc.robot.commands.Autos.Competition.Pony;
import frc.robot.commands.Autos.Competition.Thoroughbred;
import frc.robot.commands.Autos.Warehouse.FourNoteAuto;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ABlinkin;
import frc.robot.subsystems.Climber;
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

  // TODO: Replace with Live Auto Notes (needs checking)
  private Pose2d[] notePoseList = {
      new Pose2d(2.9, 4.11, Rotation2d.fromDegrees(150)),
      new Pose2d(2.9, 5.55, Rotation2d.fromDegrees(180)),
      new Pose2d(2.9, 7.0, Rotation2d.fromDegrees(-150)),
      new Pose2d(8.29, 7.46, Rotation2d.fromDegrees(180))
  };

  // TODO: Replace with Live Shoot positions (3 woofer locations, and left & right
  // wing locations)
  private Pose2d[] shootPoseList = {
      new Pose2d(2.4, 6.22, Rotation2d.fromDegrees(180)),
      MMField.getBlueWooferApproachPose(),
  };

  // Controllers
  public MMController driverController = new MMController(0)
      .setDeadzone(.1 / 2)
      .setScaleXLeft(-MaxSpeed)
      .setScaleYLeft(-MaxSpeed)
      .setScaleXRight(-MaxAngularRate);

  public MMController oppController = new MMController(1)
      .setDeadzone(.1 / 2)
      .setScaleXLeft(-MaxSpeed)
      .setScaleYLeft(-MaxSpeed)
      .setScaleXRight(-MaxAngularRate);

  public CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
  SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // Telemetry logger = new Telemetry(MaxSpeed);

  // Subsystems
  public Shooter shooterSubsystem = new Shooter(this);
  public Navigation navigation = new Navigation(this);
  public Climber climber = new Climber(this);
  public ABlinkin aBlinkin = new ABlinkin(this);

  private final SendableChooser<Command> autoChooser;
  public static SendableChooser<Pose2d> startPoseChooser;
  public static SendableChooser<Pose2d> noteChooser0;
  public static SendableChooser<Pose2d> shootChooser0;
  public static SendableChooser<Pose2d> noteChooser1;
  public static SendableChooser<Pose2d> shootChooser1;
  public static SendableChooser<Pose2d> noteChooser2;
  public static SendableChooser<Pose2d> shootChooser2;
  public static SendableChooser<Pose2d> noteChooser3;
  public static SendableChooser<Pose2d> shootChooser3;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private void configureBindings() {

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive
            .withVelocityX(driverController.getLeftYSmoothed() * Robot.resetDriverValue)
            .withVelocityY(driverController.getLeftXSmoothed() * Robot.resetDriverValue)
            .withRotationalRate(driverController.getRightXSmoothed())));

    // TODO: Low Priority Driver Controller Layout
    // Needs:
    // -Shoot : RT
    // -Eject
    // -Intake
    // -Aim
    // -Reset State Machine
    // -Reset Position
    // -Aim
    // -Climb / StopClimb
    // -RequestAmp(shoot/ eject are the same thing)
    // -Clean up comments please

    // driverController.rightBumper().whileTrue(new ChaseAndIntakeBroken(this));
    // driverController.rightTrigger().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setShootFlag(true)))
    // .onFalse(new InstantCommand(() -> shooterSubsystem.setShootFlag(false)));
    // driverController.a().whileTrue(new Aim(this));
    // driverController.b().whileTrue(new InstantCommand(() ->
    // shooterSubsystem.setIntakeFlag(true)))
    // .onFalse(new InstantCommand(() -> shooterSubsystem.setIntakeFlag(false)));
    // driverController.leftTrigger().onTrue(new GoShoot(this));

    // driverController.leftBumper().onTrue(
    // new ParallelCommandGroup(drivetrain.runOnce(() ->
    // drivetrain.seedFieldRelative())));
    // driverController.button(8)
    // .onTrue(new InstantCommand(() ->
    // drivetrain.seedFieldRelative(MMField.currentWooferPose())));

    // driverController.y().whileTrue(new GoAmp(this));

    // driverController.x().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setElevatorIndexFlag(true)));
    // driverController.povDown()
    // .onTrue(new ParallelCommandGroup(new InstantCommand(() ->
    // shooterSubsystem.resetStateMachine()),
    // new InstantCommand(() -> climber.resetStateMachine())));
    // driverController.povUp().whileTrue(new AimToWall(this));
    // driverController.povRight().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setChuckFlag(true)));
    // driverController.povLeft().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setShootOverrideFlag(true)));
    // driverController.povUp().whileTrue( new InstantCommand(() ->
    // shooterSubsystem.shooterAngleMargin+=.0001));
    // driverController.povUp().whileTrue( new InstantCommand(() ->
    // shooterSubsystem.shooterAngleMargin-=.0001));
    // driverController.getHID().setRumble(null, MaxAngularRate);

    // driverController.leftTrigger().whileTrue(new InstantCommand(()->
    // shooterSubsystem.runElevatorBeltUpSlow()));
    // joystick.x().whileTrue(new PathFindTo(this,
    // MMField::getBlueWooferApproachPose));

    // oppController.a().onTrue(new StartClimb(this));
    // oppController.a().onTrue(new InstantCommand(() ->
    // climber.setClimbUnwindFlag(true)));
    // oppController.button(8).onTrue(new InstantCommand(() ->
    // shooterSubsystem.setRunDiagnosticFlag(true)));

    // oppController.povUp().whileTrue(new FullClimb(this,
    // MMField.getBlueStageFieldPose()));
    // oppController.povLeft().whileTrue(new FullClimb(this,
    // MMField.getBlueStageSpeakerSidePose()));
    // oppController.povRight().whileTrue(new FullClimb(this,
    // MMField.getBlueStageNonSpeakerSidePose()));
    // oppController.povUp().whileTrue(new FullClimb(this,
    // MMField.getBlueStageFieldPose()));
    // oppController.povLeft().whileTrue(new FullClimb(this,
    // MMField.getBlueStageSpeakerSidePose()));
    // oppController.povRight().whileTrue(new FullClimb(this,
    // MMField.getBlueStageNonSpeakerSidePose()));
    // oppController.leftBumper().onTrue(new InstantCommand(() ->
    // climber.setMoveHooksUp(true)));
    // oppController.rightBumper().onTrue(new StartClimbManual(this));
    // oppController.povDown().onTrue(new InstantCommand(() ->
    // navigation.resetVision()));

    // oppController.povLeft().whileTrue(new SetColor
    // (this, "Intake"));
    // oppController.povRight().whileTrue(new SetCol

    // oppController.b().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setReverseIntakeFlag(true)));
    // oppController.leftTrigger()
    // .onTrue(new InstantCommand(() ->
    // MMFiringSolution.decrementManualChangeAngle()));
    // oppController.rightTrigger()
    // .onTrue(new InstantCommand(() ->
    // MMFiringSolution.incrementManualChangeAngle()));
    // oppController.button(10)
    // .onTrue(new InstantCommand(() -> MMFiringSolution.resetManualChangeAngle()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    // drivetrain.registerTelemetry(logger::telemeterize);

    // Final CONTROLS:
    driverController.x().onTrue(new InstantCommand(() -> shooterSubsystem.setElevatorIndexFlag(true)));
    driverController.b().whileTrue(new InstantCommand(() -> shooterSubsystem.setIntakeFlag(true)));
    driverController.a().onTrue(new InstantCommand(() -> shooterSubsystem.setReverseIntakeFlag(true)));
    driverController.y().whileTrue(new GoAmp(this));
    driverController.povDown()
        .onTrue(new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.resetStateMachine()),
            new InstantCommand(() -> climber.resetStateMachine())));

    driverController.rightBumper().whileTrue(new ChaseAndIntakeBroken(this, true));
    // driverController.rightTrigger().onTrue(new InstantCommand(() ->
    // shooterSubsystem.setChuckLowFlag(true)));
    driverController.leftTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setChuckLowFlag(true)));
    driverController.rightTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setChuckHighFlag(true)));
    driverController.leftBumper().onTrue(new SignalForNote(this));

    oppController.a().whileTrue(new Aim(this));
    oppController.y().onTrue(new ClawsUpAndIndex(this));
    oppController.x().onTrue(new StartClimbManual(this));
    oppController.b().onTrue(new InstantCommand(() -> navigation.resetVision()));

    oppController.rightTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setShootFlag(true)))
        .onFalse(new InstantCommand(() -> shooterSubsystem.setShootFlag(false)));
    // oppController.leftTrigger().whileTrue(new FullChuckLow(this));
    oppController.leftTrigger().onTrue(new InstantCommand(() -> shooterSubsystem.setWooferSlamFlag(true)));

    // oppController.leftBumper().whileTrue(new
    // InstantCommand(()->shooterSubsystem.aimToWall()));
    oppController.rightBumper().whileTrue(new InstantCommand(() -> shooterSubsystem.setChuckLowFlag(true)));
    // oppController.rightBumper().onTrue(new InstantCommand(()->
    // shooterSubsystem.));

    oppController.button(7).onTrue(new InstantCommand(() -> climber.setClimbUnwindFlag(true)));

    oppController.button(8).onTrue(new InstantCommand(() -> shooterSubsystem.setRunDiagnosticFlag(true)));
    oppController.povUp().whileTrue(new FullClimb(this,
        MMField.getBlueStageFieldPose()));
    oppController.povLeft().whileTrue(new FullClimb(this,
        MMField.getBlueStageSpeakerSidePose()));
    oppController.povRight().whileTrue(new FullClimb(this,
        MMField.getBlueStageNonSpeakerSidePose()));
    oppController.leftBumper().whileTrue(new AimToWall(this));
    oppController.button(10)
        .onTrue(new ParallelCommandGroup(new InstantCommand(() -> shooterSubsystem.resetStateMachine()),
            new InstantCommand(() -> climber.resetStateMachine())));

  }

  public RobotContainer() {

    configureBindings();

    //
    // PathPlanner
    //
    // Register named commands
    NamedCommands.registerCommand("dropCone", new ShootTheConeOut(this));

    // Set Up Autochooser
    // Default auto will be `Commands.none()`
    // autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("none", Commands.none());
    autoChooser.addOption("FourNoteAuto-Shop", new FourNoteAuto(this));
    autoChooser.addOption("ShootSmove-Shop", new AutoSamplerShootSmove(this));
    autoChooser.addOption("MustangAuto-Shop", new MustangAuto(this));
    autoChooser.addOption("StageSideAuto-Shop", new StageSideAuto(this));
    autoChooser.addOption("ArabianAuto-Comp", new Arabian(this));
    autoChooser.addOption("Thoroughbred-Comp", new Thoroughbred(this));
    autoChooser.addOption("Horseshoe2-Comp", new HorseShoeTwo(this));
    autoChooser.addOption("Pony-Comp", new Pony(this));
    autoChooser.addOption("CrazyHorse-Comp", new CrazyHorse(this));

    // autoChooser.addOption("FourNoteAuto", new badAuto(this));

    autoChooser.addOption("HorseShoeAuto-Comp", new HorseShoe(this));
    autoChooser.setDefaultOption("none", Commands.none());
    SmartDashboard.putData("Auto Mode", autoChooser);

    // noteChooser0 = new SendableChooser<Pose2d>();
    // noteChooser1 = new SendableChooser<Pose2d>();
    // shootChooser0 = new SendableChooser<Pose2d>();
    // shootChooser1 = new SendableChooser<Pose2d>();

    noteChooser0 = fillNoteChooser("Note 1");
    noteChooser1 = fillNoteChooser("Note 2");
    noteChooser2 = fillNoteChooser("Note 3");
    noteChooser3 = fillNoteChooser("Note 4");

    shootChooser0 = fillShootPoseChooser("Shoot Pose 1");
    shootChooser1 = fillShootPoseChooser("Shoot Pose 2");
    shootChooser2 = fillShootPoseChooser("Shoot Pose 3");
    shootChooser3 = fillShootPoseChooser("Shoot Pose 4");

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

  // Removed as Unused Code...
  // // strip this down to use none of our code...
  // // hard code the poses, etc.
  // // Michael Jansen says it is our code. Let's find out.
  // public Command runDeferredTest() {
  // PathConstraints trajectoryConstraints = new PathConstraints(1.5, 3, 2 *
  // Math.PI, 4 * Math.PI);
  // List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
  // new Pose2d(new Translation2d(1.8, 5.5), Rotation2d.fromDegrees(180)),
  // new Pose2d(new Translation2d(1.6, 5.5), Rotation2d.fromDegrees(180)),
  // new Pose2d(new Translation2d(1.4, 5.5), Rotation2d.fromDegrees(180)));
  // PathPlannerPath path = new PathPlannerPath(bezierPoints,
  // trajectoryConstraints,
  // new GoalEndState(0, Rotation2d.fromDegrees(180)));
  // path.preventFlipping = false;
  // return AutoBuilder.followPath(path);
  // }
}