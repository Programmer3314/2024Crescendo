// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;
import frc.robot.MMUtilities.MMTurnPIDController;
import frc.robot.MMUtilities.MMWaypoint;
import frc.robot.commands.SetColor;

public class Shooter extends SubsystemBase {
  RobotContainer rc;

  public static HashMap<String, MMWaypoint> determineShot = new HashMap<String, MMWaypoint>();

  private ShooterStateMachine ssm = new ShooterStateMachine();
  private MMTurnPIDController turnPidController = new MMTurnPIDController(true);
  private MMTurnPIDController turnWallPidController = new MMTurnPIDController(true);
  public Rotation2d targetAngleSpeaker;
  Rotation2d leftBoundaryAngleSpeaker;
  Rotation2d rightBoundaryAngleSpeaker;
  Pose2d speakerPose;
  Pose2d currentPose;
  double speakerTurnRate;
  double WallTurnRate;

  public double shooterAngleMargin = .002;
  double shooterVelocityMargin = 2;
  double intakeVelocityMargin = 20;
  double intakeRotationMargin = .24;
  double elevatorPositionMargin = 1;
  double leftShooterChuckLowVelocity = 30;
  double rightShooterChuckLowVelocity = 40;
  double leftShooterRidLowVelocity = 10;
  double rightShooterRidLowVelocity = 15;
  double shooterChuckLowRotation = .38;
  double leftShooterChuckHighVelocity = 45;
  double rightShooterChuckHighVelocity = 55;
  double shooterChuckHighRotation = .43;
  double leftShooterWooferSlamVelocity = 35;
  double rightShooterWooferSlamVelocity = 45;
  double shooterAngleWooferSlam = .45;

  int abortIntakeCounter;

  boolean runAim;
  boolean runWallAim;
  boolean runIntake;
  boolean abortIntake;
  boolean runElevatorIndex;
  boolean runShoot;
  boolean runOutTake;
  double shooterDelay = .125;
  double outTakeDelay = .125;
  boolean runChuckLow;
  boolean runChuckHigh;
  boolean runWooferSlam;
  boolean runShootOverride;
  boolean manualAngleIncrement;
  boolean manualAngleDecrement;

  double shotStartTime;
  double shotEndTime;
  double shotTotalTime;

  boolean runDiagnosticTest;
  double diagnosticRunTime = 3;
  double diagnosticTimeOut = 20;
  double diagnosticShooterAngle = .41;
  double diagnosticLeftMotorSpeed = 4;
  double diagnosticRightMotorSpeed = diagnosticLeftMotorSpeed;
  double DiagnosticElevatorBeltVel = 10;
  double DiagnosticElevatorBeltMargin = 5;
  double autoShooterAngle;
  double autoShotChange = .004;
  double autoShooterLeftVelocity;
  double autoShooterRightVelocity;

  boolean diagnosticDesiredIntakeUp;
  boolean diagnosticDesiredIntakeDown;
  boolean diagnosticDesiredIntakeIn;
  boolean diagnosticMoveToIndex;
  boolean diagnosticDesiredIndexIn;
  boolean diagnosticDesiredIndexOut;
  boolean diagnosticDesiredShooterAngle;
  boolean diagnosticDesiredRightShooterVel;
  boolean diagnosticDesiredElevatorUp;
  boolean diagnosticDesiredElevatorDown;
  boolean diagnosticDesiredElevatorBeltUp;
  boolean diagnosticDesiredElevatorBeltDown;
  boolean diagnosticDesiredLeftShooterVel;
  boolean autoForceShot;
  boolean autoStartForceAngle;
  boolean shootAutoShot;
  boolean intakeOverNote;

  double elevatorHomingVelocity = -20;
  boolean hasHomedElevator;

  // public Orchestra orchestra = new Orchestra();

  public MMWaypoint desiredWaypoint;

  public MMFiringSolution firingSolution;

  private TalonFX intakeBeltMotor = new TalonFX(9, "CANIVORE");
  private TalonFX intakeRotateMotor = new TalonFX(10, "CANIVORE");
  private TalonFX shooterRotateMotor = new TalonFX(11, "CANIVORE");
  private TalonFX index1 = new TalonFX(12, "CANIVORE");
  private TalonFX index2 = new TalonFX(13, "CANIVORE");
  private TalonFX leftMotor = new TalonFX(14, "CANIVORE");
  private TalonFX rightMotor = new TalonFX(15, "CANIVORE");
  private TalonFX elevatorMotor = new TalonFX(16, "CANIVORE");
  private TalonFX elevatorBottomBelt = new TalonFX(17, "CANIVORE");
  private TalonFX elevatorTopBelt = new TalonFX(18, "CANIVORE");

  CANcoder intakeRotateCanCoder = new CANcoder(5, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(6, "CANIVORE");

  DigitalInput intakeBreakBeam = new DigitalInput(2);// NOTE: broken = false, solid = true
  DigitalInput shooterBreakBeam = new DigitalInput(1);
  DigitalInput elevatorHomeSensor = new DigitalInput(0);
  DigitalInput elevatorBreakBeam = new DigitalInput(3);

  double intakeTop = .823;
  double intakeUpPos = intakeTop - .005;
  double intakeDownPos = intakeTop - .75;// - .74

  double intakeVelIn = 30;
  double intakeVelOut = -20;

  double elevatorInPerRev = 5.125 / 30;
  double elevatorDownPosition = .1;
  double elevatorAmpPosition = 47.2;
  double elevatorTrapShootPosition = 46;
  double elevatorTrapPosition = 67.0;

  double shooterAngleDownPos = .38;

  double ampUpPosition = 37.2;

  double index1InVel = 8;
  double index2InVel = index1InVel;

  int shotCounter = 0;
  int indexCounter = 0;
  int reverseCounter = 0;
  int idleCounter = 0;

  double index1OutVel = -30;
  double index2OutVel = index1OutVel;

  private final MotionMagicVoltage shooterRotateMotionMagicVoltage = new MotionMagicVoltage(0);
  private final MotionMagicVoltage elevatorMotorMotionMagicVoltage = new MotionMagicVoltage(0);

  // private final PositionVoltage testShooterPositionVoltage = new
  // PositionVoltage(0);
  private final MotionMagicVoltage intakeRotateMotionMagicVoltage = new MotionMagicVoltage(0);
  private VelocityVoltage index1VelVol = new VelocityVoltage(0);
  private VelocityVoltage index2VelVol = new VelocityVoltage(0);
  private VelocityVoltage leftVelVol = new VelocityVoltage(0);
  private MotionMagicVelocityVoltage leftMMVelVol = new MotionMagicVelocityVoltage(0);
  private MotionMagicVelocityVoltage rightMMVelVol = new MotionMagicVelocityVoltage(0);
  private VelocityVoltage rightVelVol = new VelocityVoltage(0);
  private VelocityVoltage intakeBeltVelVol = new VelocityVoltage(0);
  private VoltageOut elevatorVoltageOut = new VoltageOut(0);
  private VelocityVoltage elevatorVelVol = new VelocityVoltage(0);

  StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("currentPose", Pose2d.struct).publish();
  StringLogEntry shooterStateLog;
  BooleanLogEntry intakeFlagLog;
  BooleanLogEntry aimFlagLog;
  DoubleLogEntry intakePositionLog;
  DoubleLogEntry intakeVelocityLog;
  DoubleLogEntry leftIndexVelocityLog;
  DoubleLogEntry rightIndexVelocityLog;
  DoubleLogEntry shooterRotatePositionLog;
  DoubleLogEntry leftShooterVelocityLog;
  DoubleLogEntry rightShooterVelocityLog;
  BooleanLogEntry intakeBreakBeamLog;
  BooleanLogEntry shooterBreakBeamLog;
  BooleanLogEntry atTargetAngleLog;
  BooleanLogEntry atSpeedLog;
  BooleanLogEntry atShooterAngleLog;
  BooleanLogEntry autoForceLog;
  BooleanLogEntry autoShootLog;
  String diagnosticState = "None";

  /** Creates a new Shooter. */
  public Shooter(RobotContainer rc) {
    this.rc = rc;
    // sets up our targets for the auto shots
    determineShot.put("arabian_2", new MMWaypoint(0, .391, 37, 53, 40));
    determineShot.put("arabian_3", new MMWaypoint(0, .391, 37, 53, 40));
    determineShot.put("pony_2", new MMWaypoint(0, .395, 37, 53, 40));
    determineShot.put("pony_3", new MMWaypoint(0, .395, 37, 53, 40));
    determineShot.put("peddie_pony_2", new MMWaypoint(0, .390, 37, 53, 40));
    determineShot.put("peddie_pony_3", new MMWaypoint(0, .390, 37, 53, 40));
    determineShot.put("Horseshoe2_5", new MMWaypoint(0, .386, 37, 53, 40));
    determineShot.put("thoroughbred_3", new MMWaypoint(0, .378, 37, 53, 40));
    determineShot.put("thoroughbred_4", new MMWaypoint(0, .378, 37, 53, 40));
    determineShot.put("crazyhorse_1", new MMWaypoint(0, .41, 37, 53, 40));
    determineShot.put("crazyhorse_2", new MMWaypoint(0, .406, 37, 53, 40));
    determineShot.put("crazyhorse_3", new MMWaypoint(0, .41, 37, 53, 40));
    determineShot.put("crazyhorse_4", new MMWaypoint(0, .404, 37, 53, 40));
    determineShot.put("crazyhorse_5", new MMWaypoint(0, .378, 47, 63, 55));

    firingSolution = new MMFiringSolution(
        rc,
        new MMWaypoint(1.3, .45, 32, 48, 48),
        new MMWaypoint(2.12, .425, 37, 53, 53),
        new MMWaypoint(2.81, 0.394, 37, 53, 53),
        new MMWaypoint(3.55, .39, 37, 53, 53),
        new MMWaypoint(4.5, .38, 47, 63, 63),
        new MMWaypoint(4.78, .379, 47, 63, 63));
    configShooterRotateCanCoder();
    configShooterRotateMotor();
    configIntakeRotateCanCoder();
    configIntakeRotateMotor();
    configIntakeBeltMotor();
    configShooterMotors();
    configIndexMotors();
    configElevatorMotor();
    ssm.setInitial(ssm.Start);

    SmartDashboard.putData("Run Diagnostic",
        new InstantCommand(() -> this.setRunDiagnosticFlag(true)));
    // SmartDashboard.putData("Run Belt Up Fast",
    // new InstantCommand(() -> this.runElevatorBeltUpFast()));
    // SmartDashboard.putData("Run Belt Up Slow",
    // new InstantCommand(() -> this.runElevatorBeltUpSlow()));
    // SmartDashboard.putData("Run Belt Down Fast",
    // new InstantCommand(() -> this.runElevatorBeltDownFast()));
    // SmartDashboard.putData("Run Belt Down Slow",
    // new InstantCommand(() -> this.runElevatorBeltDownSlow()));
    DataLog log = DataLogManager.getLog();
    shooterStateLog = new StringLogEntry(log, "/my/state/shooter");
    intakeFlagLog = new BooleanLogEntry(log, "my/flag/intake");
    aimFlagLog = new BooleanLogEntry(log, "my/flag/aim");
    intakePositionLog = new DoubleLogEntry(log, "my/intake/position");
    intakeVelocityLog = new DoubleLogEntry(log, "my/intake/velocity");
    leftIndexVelocityLog = new DoubleLogEntry(log, "my/shooter/leftIndexVel");
    rightIndexVelocityLog = new DoubleLogEntry(log, "my/shooter/rightIndexVel");
    shooterRotatePositionLog = new DoubleLogEntry(log, "my/shooter/position");
    leftShooterVelocityLog = new DoubleLogEntry(log, "my/shooter/leftVel");
    rightShooterVelocityLog = new DoubleLogEntry(log, "my/shooter/rightVel");
    intakeBreakBeamLog = new BooleanLogEntry(log, "my/breakBeam/intake");
    shooterBreakBeamLog = new BooleanLogEntry(log, "my/breakBeam/shooter");
    atTargetAngleLog = new BooleanLogEntry(log, "my/shooterCondition/targetAngle");
    atSpeedLog = new BooleanLogEntry(log, "my/shooterCondition/motorSpeed");
    atShooterAngleLog = new BooleanLogEntry(log, "my/shooterCondition/shooterAngle");
    autoForceLog = new BooleanLogEntry(log, "my/autoShooterCondition/forceShot");
    autoShootLog = new BooleanLogEntry(log, "my/autoShooterCondition/shoot");

    // var status = orchestra.loadMusic("rickroll.chrp");
    // orchestra.addInstrument(rc.climber.climbMotor);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Played Time", orchestra.getCurrentTime());
    // ChassisSpeeds chassisSpeeds = rc.drivetrain.getCurrentRobotChassisSpeeds();

    // rc.navigation.updatePredictedPosition(chassisSpeeds.vxMetersPerSecond,
    // chassisSpeeds.vyMetersPerSecond,
    // chassisSpeeds.omegaRadiansPerSecond);

    calcFiringSolution();
    // calcPredictedFiringSolution();
    speakerPose = MMField.currentSpeakerPose();
    currentPose = rc.drivetrain.getState().Pose;

    Translation2d transformFromSpeaker = speakerPose.getTranslation().minus(currentPose.getTranslation());

    // predictedTargetAngleSpeaker =
    targetAngleSpeaker = transformFromSpeaker.getAngle();
    Translation2d transformLeftBoundarySpeaker = MMField.currentLeftBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    leftBoundaryAngleSpeaker = transformLeftBoundarySpeaker.getAngle();
    Translation2d transformRightBoundarySpeaker = MMField.currentRightBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    rightBoundaryAngleSpeaker = transformRightBoundarySpeaker.getAngle();
    // TODO: fix abort intake
    if (intakeRotateMotor.getSupplyCurrent().getValue() > 2.1) {
      // abortIntake = true;
      abortIntakeCounter++;
    }

    SmartDashboard.putNumber("changed angle", MMFiringSolution.manualChangeAngle);

    SmartDashboard.putString("Check NonSpeakerApproach", MMField.currentStageNonSpeakerPose().toString());
    SmartDashboard.putString("Check SpeakerApproach", MMField.currentStageSpeakerSidePose().toString());
    SmartDashboard.putString("Check FieldApproach", MMField.currentStagePose().toString());

    SmartDashboard.putBoolean("Elevator Home", elevatorHomeSensor.get());

    // TODO: finish below diagnostic (null)
    SmartDashboard.putString("DGState", diagnosticState);
    // if (!hasHomedElevator) {
    // runElevatorToHome();
    // }
    // if (!elevatorHomeSensor.get()) {
    // // elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(0));
    // elevatorMotor.setPosition(0);
    // hasHomedElevator = true;
    // }
    // }

    logUpdate();

    ssm.update();

    // TODO: Check this for Red.
    turnPidController.initialize(targetAngleSpeaker);
    speakerTurnRate = turnPidController.execute(currentPose.getRotation());

    // TODO: This might not work for red. We may need to
    turnWallPidController.initialize(180);
    WallTurnRate = turnWallPidController.execute(currentPose.getRotation());
    // if (runWallAim) {
    // aimToWall();
    // }
    // if (runAim) {
    // aimToSpeaker();
    // }
    // // aimToSpeakerNoShoot();
    // } else if (runChuck) {
    // aimForChuck();
    // } else if (runWooferSlam) {
    // aimForWooferSlam();
    // }

    // else {
    // // stopShooterMotors();
    // }
    if (runDiagnosticTest) {
      SmartDashboard.putBoolean("diagnosticIntakeDown", diagnosticDesiredIntakeDown);
      SmartDashboard.putBoolean("diagnosticIntakeIn", diagnosticDesiredIntakeIn);
      SmartDashboard.putBoolean("diagnosticIntakeOut", diagnosticMoveToIndex);
      SmartDashboard.putBoolean("diagnosticIntakeUp", diagnosticDesiredIntakeUp);
      SmartDashboard.putBoolean("diagnosticIndexIn", diagnosticDesiredIndexIn);
      SmartDashboard.putBoolean("diagnosticIndexOut", diagnosticDesiredIndexOut);
      SmartDashboard.putBoolean("diagnosticShooterAngle", diagnosticDesiredShooterAngle);
      SmartDashboard.putBoolean("diagnosticShooterVel", diagnosticDesiredRightShooterVel);
      SmartDashboard.putBoolean("diagnosticShooterVel", diagnosticDesiredLeftShooterVel);
    }
    SmartDashboard.putBoolean("Shooter Beam", shooterBreakBeam.get());
    SmartDashboard.putBoolean(" Not Move Intake Beam", intakeBreakBeam.get());
    SmartDashboard.putString("State Machine State", currentStateName());
    SmartDashboard.putBoolean("AbortIntakeFlag", abortIntake);
    SmartDashboard.putNumber("AbortIntakeCounter", abortIntakeCounter);
    SmartDashboard.putNumber("TotalShotTime", shotTotalTime);
    SmartDashboard.putBoolean("Elevator Break Beam", elevatorBreakBeam.get());
  }

  public class ShooterStateMachine extends MMStateMachine {

    MMStateMachineState Start = new MMStateMachineState("Start") {

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return Index;
        }
        if (!intakeBreakBeam.get()) {
          return DropIntake;
        }
        if (!elevatorBreakBeam.get()) {
          return ElevatorIndexed;
        }
        return Idle;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
        // setIntakeFlag(false);
        // setShootFlag(false);
        // setAimFlag(false);
        rc.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        rc.oppController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
    };

    MMStateMachineState Idle = new MMStateMachineState("Idle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setAutoStartForceAngle(false);
        setReadyToAutoShoot(false);
        setIntakeUp();
        stopIndexers();
        stopShooterMotors();
        stopIntake();
        setRunDiagnosticFlag(false);
        stopElevatorBelts();
        setElevatorDown();
        if (!intakeOverNote) {
          setIntakeFlag(false);
        } else {
          intakeOverNote = false;
        }
        setShootFlag(false);
        setAimFlag(false);
        setAimWallFlag(false);
        setAbortIntakeFlag(false);
        setReverseIntakeFlag(false);
        setChuckLowFlag(false);
        setShootOverrideFlag(false);
        setWooferSlamFlag(false);
        setElevatorIndexFlag(false);
        setAutoForce(false);
        shooterDown();
        // abortIntakeCounter=0;
        idleCounter++;
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (runIntake) {
          return DropIntake;
        }
        if (runDiagnosticTest) {
          return DiagnosticSetIntakeDown;
        }
        if (runOutTake) {
          return IntakeReverse;
        }
        return this;
      }
    };

    MMStateMachineState DropIntake = new MMStateMachineState("Drop Intake") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();
        runIntakeIn();

        rc.driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
        rc.oppController.getHID().setRumble(RumbleType.kBothRumble, 1);

        rc.aBlinkin.controlBlink(rc.aBlinkin.intakeBlinkValue);
      }

      @Override
      public MMStateMachineState calcNextState() {
        // if (!shooterBreakBeam.get()) {
        // return Index;
        // }
        if (abortIntakeCounter > 1) {
          return IntakeStallPause;
        }
        if (!intakeBreakBeam.get()) {
          return intakeBroken;
        }
        return this;
      }
    };

    MMStateMachineState intakeBroken = new MMStateMachineState("Intake Broken") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();
        // runIntakeIn();
        runIntakeInSlow();
        // setAimFlag(true); Changed to stop battery bleed

        rc.aBlinkin.controlBlink(0.77);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return Index;
        }
        if (!intakeBreakBeam.get()) {
          return this;
        }
        if (abortIntake) {
          return Idle;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState nexState) {
        setIntakeUp();
      }
    };

    MMStateMachineState Index = new MMStateMachineState("Index") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        stopIndexers();
        setIntakeFlag(false);
        // setAimFlag(true); BB
        stopElevatorBelts();
        setShootFlag(false);
        setAimFlag(false);
        setAimWallFlag(false);
        setChuckLowFlag(false);
        setShootOverrideFlag(false);
        setWooferSlamFlag(false);
        setElevatorIndexFlag(false);
        indexCounter++;

        rc.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        rc.oppController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (runElevatorIndex && isInMargin(intakeRotateCanCoder.getAbsolutePosition().getValue(),
            intakeUpPos, intakeRotationMargin)) {
          return ElevatorDown;
        }
        if (autoForceShot) {
          return PrepareToAutoShoot;
        }
        if (runShoot) {
          return PrepareToShoot;
        }
        if (runChuckLow) {
          return PrepareToChuckLow;
        }
        if (runChuckHigh) {
          return PrepareToChuckHigh;
        }
        if (runWooferSlam) {
          return PrepareToWooferSlam;
        }
        if (runOutTake) {
          return IntakeReverse;
        }
        return this;
      }
    };
    MMStateMachineState PrepareToAutoShoot = new MMStateMachineState("PrepareToAutoShoot") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // aimToWall();
        aimForAuto();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isReadyToAutoShoot() && shootAutoShot) {
          return Shoot;
        }
        return this;
      }
    };
    MMStateMachineState PrepareToChuckLow = new MMStateMachineState("PrepareToChuckLow") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // aimToWall();
        aimForChuckLow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (readyToChuck() && runChuckLow) {// TODO: is this really needed, the flag should already be on
          return ChuckShoot;
        }
        return this;
      }
    };

    MMStateMachineState PrepareToChuckHigh = new MMStateMachineState("PrepareToChuckHigh") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // aimToWall();
        aimForChuckHigh();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (readyToChuckHigh() && runChuckHigh) {
          return ChuckShoot;
        }
        return this;
      }
    };

    MMStateMachineState PrepareToWooferSlam = new MMStateMachineState("PrepareToWooferSlam") {
      public void transitionTo(MMStateMachineState previousState) {
        aimForWooferSlam();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (readyToWooferSlam() && runWooferSlam) {// TODO: is this really needed, the flag should already be on
          return WooferSlam;
        }
        return this;
      }
    };

    MMStateMachineState PrepareToShoot = new MMStateMachineState("PrepareToShoot") {
      public void transitionTo(MMStateMachineState previousState) {
      }

      @Override
      public void doState() {
        aimToSpeaker();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if ((readyToShoot() && runShoot) || runShootOverride) {
          return Shoot;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorDown = new MMStateMachineState("ElevatorDown") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorDown();
        setAimFlag(false);
        setAimWallFlag(false);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isInMargin(elevatorMotor.getPosition().getValue(), elevatorDownPosition, elevatorPositionMargin)
            || !elevatorHomeSensor.get()) {
          return ElevatorIndex;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState previousState) {
        setElevatorZero();
      }
    };

    MMStateMachineState ElevatorIndex = new MMStateMachineState("ElevatorIndex") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBottomBeltUpFast();
        runIntakeOut();
        runIndexOut();
        stopShooterMotors();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!elevatorBreakBeam.get()) {
          return ElevatorIndexed;// ElevatorPassNoteAbove2
        }
        return this;
      }
    };

    MMStateMachineState ElevatorPassNoteAbove2 = new MMStateMachineState("ElevatorPassNoteAbove2") {
      double rev;

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rev = elevatorBottomBelt.getPosition().getValue();
        runElevatorBottomBeltUpSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (Math.abs(elevatorBottomBelt.getPosition().getValue() - rev) > 4.5) {
          return ElevatorIndexed;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorIndexed = new MMStateMachineState("Elevator Indexed") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorIndexFlag(false);
        stopElevatorBelts();
        stopIndexers();
        stopIntake();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (runAim) {
          return ElevatorPassNoteAbove;
        }
        if (runOutTake) {
          return ElevatorDownToIndex;
        }
        if (runIntake) {
          return ElevatorDownAbort;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nexState) {
        setReverseIntakeFlag(false);
      }
    };

    MMStateMachineState ElevatorPassNoteAbove = new MMStateMachineState("ElevatorPassNoteAbove") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBottomBeltUpSlow();
        runElevatorTopBeltUpSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (elevatorBreakBeam.get()) {
          return SetElevatorHeight;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nexState) {
        stopElevatorBelts();
      }
    };

    MMStateMachineState SetElevatorHeight = new MMStateMachineState("SetElevatorHeight") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorUp();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isInMargin(elevatorMotor.getPosition().getValue(), elevatorAmpPosition, elevatorPositionMargin) && runShoot) {
          return ElevatorShoot;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorShoot = new MMStateMachineState("ElevatorShoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBeltShoot();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= .5) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorDownAbort = new MMStateMachineState("ElevatorDownAbort") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorDown();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isInMargin(elevatorMotor.getPosition().getValue(), elevatorDownPosition, elevatorPositionMargin)) {
          return ElevatorDownToIndex;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorDownToIndex = new MMStateMachineState("ElevatorDownToIndex") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBottomBeltDownFast();
        runIntakeIn();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return Index;
        }
        return this;
      }
    };

    MMStateMachineState Shoot = new MMStateMachineState("Shoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIntakeIn();
        runIndexShoot();
        setShootFlag(false);
        setShotStartTime();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return ShootPauseBroken;
        }
        if (shooterBreakBeam.get()) {
          return ShootPause;
        }
        return this;
      }
    };

    MMStateMachineState WooferSlam = new MMStateMachineState("WooferSlam") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIntakeIn();
        runIndexShoot();
        setShootFlag(false);
        setWooferSlamFlag(false);
        setShotStartTime();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return ShootPauseBroken;
        }
        if (shooterBreakBeam.get()) {
          return ShootPause;
        }
        return this;
      }
    };

    MMStateMachineState ChuckShoot = new MMStateMachineState("ChuckLow") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIntakeIn();
        runIndexShoot();
        setShootFlag(false);
        setChuckLowFlag(false);
        setChuckHighFlag(false);
        setShotStartTime();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return ShootPauseBroken;
        }
        if (shooterBreakBeam.get()) {
          return ShootPause;
        }
        return this;
      }
    };

    MMStateMachineState ShootPauseBroken = new MMStateMachineState("ShootPauseBroken") {
      @Override
      public MMStateMachineState calcNextState() {
        if (shooterBreakBeam.get()) {
          return ShootPause;
        }
        return this;
      }
    };

    MMStateMachineState ShootPause = new MMStateMachineState("ShootPause") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // TODO: Are the next two lines needed/used at all?
        shotEndTime = Timer.getFPGATimestamp();
        shotTotalTime = shotEndTime - shotStartTime;
        // totaltime=.1
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= shooterDelay) {
          shotCounter++;
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState IntakeReverse = new MMStateMachineState("IntakeReverse") {

      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();

      }

      @Override
      public MMStateMachineState calcNextState() {
        if (Math.abs(intakeRotateMotor.getPosition().getValue() - intakeDownPos) < .05) {
          return IndexReverse;
        }
        // return IndexReverse;
        return this;
      }
    };

    MMStateMachineState IndexReverse = new MMStateMachineState("IndexReverse") {

      public void transitionTo(MMStateMachineState previousState) {
        runIndexOut();
        runIntakeOut();
        setReverseIntakeFlag(false);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!intakeBreakBeam.get()) {
          return IntakeBrokenPause;
        }
        return this;
      }
    };

    MMStateMachineState IntakeBrokenPause = new MMStateMachineState("ShootPauseBroken") {

      @Override
      public MMStateMachineState calcNextState() {
        if (intakeBreakBeam.get()) {
          return IntakePause;
        }
        return this;
      }
    };

    MMStateMachineState IntakePause = new MMStateMachineState("IntakePause") {

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= outTakeDelay) {
          reverseCounter++;
          return Idle;
        } else {

          return this;
        }
      }
    };

    MMStateMachineState DiagnosticSetIntakeDown = new MMStateMachineState("DiagnosticSetIntakeDown") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();
        setAimFlag(false);
        diagnosticState = "IntakeDown";
        // TODO: make the first noise
        // startOrchestra();
      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeDown = isInMargin(intakeDownPos, getIntakePos(), intakeRotationMargin);

      }

      @Override
      public MMStateMachineState calcNextState() {
        if (diagnosticDesiredIntakeDown) {
          return DiagnosticIntakeIn;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticIntakeIn = new MMStateMachineState("DiagnosticIntakeIn") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIntakeIn();
        diagnosticState = "IntakeIn";
      }

      @Override
      public void doState() {
        double actualvel = getIntakeVel();
        diagnosticDesiredIntakeIn = isInMargin(intakeVelIn, actualvel, intakeVelocityMargin);

      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!intakeBreakBeam.get()) {
          return DiagnosticMoveToIndex;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;

      }
    };

    MMStateMachineState DiagnosticMoveToIndex = new MMStateMachineState("DiagnosticMoveToIndex") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexIn();
        // runIntakeOut();
        // TODO: make second noise
        // stopOrchestra();
        diagnosticState = "MoveToIndex";
      }

      @Override
      public void doState() {
        double actualvel = getIntakeVel();
        diagnosticMoveToIndex = isInMargin(intakeVelIn, actualvel, intakeVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return DiagnosticSetIntakeUpToShoot;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticShooterAngle = new MMStateMachineState("DiagnosticShooterAngle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIndexers();
        stopIntake();
        setShooterPosition(diagnosticShooterAngle);
        diagnosticState = "DiagnosticShooterAngle";
        // TODO: make Third noise
        // orchestra.play();

      }

      @Override
      public void doState() {
        diagnosticDesiredShooterAngle = isInMargin(diagnosticShooterAngle, getShooterAngle(), shooterAngleMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (diagnosticDesiredShooterAngle) {
          return DiagnosticSlowShoot;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticSlowShoot = new MMStateMachineState("DiagnosticSlowShoot") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexers(diagnosticLeftMotorSpeed, diagnosticRightMotorSpeed);
        runShooters(diagnosticLeftMotorSpeed, diagnosticRightMotorSpeed);

        diagnosticState = "DiagnosticSlowShoot";
        // TODO: make fourth noise
        // orchestra.play();

      }

      @Override
      public MMStateMachineState calcNextState() {
        if (shooterBreakBeam.get()) {
          return DiagnosticShooterAngleDown;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }

    };
    MMStateMachineState DiagnosticShooterAngleDown = new MMStateMachineState("DiagnosticShooterAngleDown") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIndexers();
        stopIntake();
        stopShooterMotors();
        setShooterPosition(shooterAngleDownPos);
        diagnosticState = "DiagnosticShooterAngleDown";
      }

      @Override
      public void doState() {
        diagnosticDesiredShooterAngle = isInMargin(shooterAngleDownPos, getShooterAngle(), shooterAngleMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (diagnosticDesiredShooterAngle) {
          return DiagnosticSlowReverse;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticSlowReverse = new MMStateMachineState("DiagnosticSlowReverse") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runShooters(-diagnosticLeftMotorSpeed, -diagnosticRightMotorSpeed);
        diagnosticState = "DiagnosticSlowReverse";
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return DiagnosticReverseToIndex;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticReverseToIndex = new MMStateMachineState("DiagnosticReverseToIndex") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexOut();
        runIntakeOut();
        // runIntakeOut();
        // TODO: make fifth noise
        diagnosticState = "DiagnosticReverseToIndex";
      }

      @Override
      public void doState() {
        double actualvel1 = getIndex1Vel();
        double actualvel2 = getIndex2Vel();
        diagnosticMoveToIndex = isInMargin(index1OutVel, actualvel1, intakeVelocityMargin)
            && isInMargin(index1OutVel, actualvel2, intakeVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!intakeBreakBeam.get()) {

          return DiagnosticMoveToElevatorIndex;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticMoveToElevatorIndex = new MMStateMachineState("DiagnosticMoveToElevatorIndex") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBottomBeltUpSlow();
        runIntakeOut();
        stopShooterMotors();
        runIndexOut();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!elevatorBreakBeam.get()) {
          return DiagnosticMoveToElevatorUp;// ElevatorPassNoteAbove2
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticMoveToElevatorUp = new MMStateMachineState("DiagnosticMoveToElevatorUp") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorUp();
        stopElevatorBelts();
        runIndexOut();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isInMargin(elevatorMotor.getPosition().getValue(), elevatorAmpPosition, elevatorPositionMargin)) {
          return DiagnosticMoveToElevatorDown;// ElevatorPassNoteAbove2
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticMoveToElevatorDown = new MMStateMachineState("DiagnosticMoveToElevatorDown") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorDown();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (isInMargin(elevatorMotor.getPosition().getValue(), elevatorDownPosition, elevatorPositionMargin)) {
          return DiagnosticMoveToIndexed;// ElevatorPassNoteAbove2
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticMoveToIndexed = new MMStateMachineState("DiagnosticMoveToIndexed") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBottomBeltDownSlow();
        runIntakeInSlow();
        runIndexIn();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!shooterBreakBeam.get()) {
          return DiagnosticReverseToIndexFinal;// ElevatorPassNoteAbove2
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticReverseToIndexFinal = new MMStateMachineState("DiagnosticReverseToIndexFinal") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexOut();
        runIntakeOut();
        // runIntakeOut();
        // TODO: make fifth noise
        diagnosticState = "DiagnosticReverseToIndex";
      }

      @Override
      public void doState() {
        double actualvel1 = getIndex1Vel();
        double actualvel2 = getIndex2Vel();
        diagnosticMoveToIndex = isInMargin(index1OutVel, actualvel1, intakeVelocityMargin)
            && isInMargin(index1OutVel, actualvel2, intakeVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!intakeBreakBeam.get()) {

          return DiagnosticSetIntakeDownEnd;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticSetIntakeDownEnd = new MMStateMachineState("DiagnosticSetIntakeDownEnd") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        stopIndexers();
        setIntakeDown();
      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeUp = isInMargin(intakeUpPos, getIntakePos(),
            intakeRotationMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (diagnosticDesiredIntakeUp) {
          return DiagnosticIntakeOut;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticIntakeOut = new MMStateMachineState("DiagnosticIntakeOut") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // TODO: make another noise
        stopShooterMotors();
        runIntakeOut();
      }

      @Override
      public void doState() {
        diagnosticMoveToIndex = isInMargin(intakeVelOut, getIntakeVel(), intakeVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (intakeBreakBeam.get() && timeInState >= diagnosticRunTime) {
          // return DiagnosticSetIntakeUp;AUtoDebug
          return Idle;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState DiagnosticSetIntakeUpToShoot = new MMStateMachineState("DiagnosticSetIntakeUpToShoot") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        stopIndexers();
        setIntakeUp();
        diagnosticState = "DiagnosticSetIntakeUpToShoot";
      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeUp = isInMargin(intakeUpPos, getIntakePos(), intakeRotationMargin);
        SmartDashboard.putBoolean("stateIntakeUpbool", diagnosticDesiredIntakeUp);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (diagnosticDesiredIntakeUp) {
          return DiagnosticShooterAngle;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };

    MMStateMachineState IntakeStallPause = new MMStateMachineState("IntakeStallPause") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        setIntakeUp();
        // setIntakeFlag(false);
        // rc.oppController.getHID().setRumble(RumbleType.kBothRumble, 0);
        // rc.driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
        rc.aBlinkin.error();
        intakeOverNote = true;
      }

      @Override
      public void doState() {
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= .5) {
          return Idle;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
        abortIntakeCounter = 0;
      }
    };
  }

  public double getLeftShooterVelocity() {
    return leftMotor.getVelocity().getValue();
  }

  public double getRightShooterVelocity() {
    return rightMotor.getVelocity().getValue();
  }

  public double getShooterAngle() {
    return shooterRotateMotor.getPosition().getValue();
  }

  public double getIntakePos() {
    return intakeRotateMotor.getPosition().getValue();
  }

  public double getIntakeVel() {
    return intakeBeltMotor.getVelocity().getValue();
  }

  public double getIndex1Vel() {
    return index1.getVelocity().getValue();
  }

  public double getIndex2Vel() {
    return index2.getVelocity().getValue();
  }

  public double getSpeakerTurnRate() {
    return speakerTurnRate;
  }

  public int getShotCounter() {
    return shotCounter;
  }

  public int getIndexCounter() {
    return indexCounter;
  }

  public int getReverseCounter() {
    return reverseCounter;
  }

  public int getIdleCounter() {
    return idleCounter;
  }

  public boolean getIntakeBreakbeam() {
    return intakeBreakBeam.get();
  }

  public double getElevatorPosition() {
    return elevatorMotor.getPosition().getValue();
  }

  public void runShooters(double leftMotorSpeed, double rightMotorSpeed) {
    runLeftMotor(leftMotorSpeed);
    runRightMotor(rightMotorSpeed);
  }

  public void runLeftMotor(double leftMotorSpeed) {
    leftMotor.setControl(leftMMVelVol.withVelocity(leftMotorSpeed));
  }

  public void runRightMotor(double rightMotorSpeed) {
    rightMotor.setControl(rightMMVelVol.withVelocity(rightMotorSpeed));
  }

  public void setShooterPosition(double position) {
    shooterRotateMotor.setControl(shooterRotateMotionMagicVoltage.withSlot(0).withPosition(position));
    // shooterRotateMotor.setControl(testShooterPositionVoltage.withPosition(position));
  }

  public void stopShooterMotors() {
    leftMotor.set(0);
    rightMotor.set(0);
  }

  public void stopLeftMotor() {
    leftMotor.set(0);
  }

  public void stopRightMotor() {
    rightMotor.set(0);
  }

  public void aimToSpeaker() {
    runLeftMotor(desiredWaypoint.getLeftVelocity());
    runRightMotor(desiredWaypoint.getRightVelocity());
    setShooterPosition(desiredWaypoint.getAngle());
  }

  // public void aimToSpeakerNoShoot() {
  // setShooterPosition(desiredWaypoint.getAngle());
  // }

  public void aimToSpeakerNoRotate() {
    runLeftMotor(desiredWaypoint.getLeftVelocity());
    runRightMotor(desiredWaypoint.getRightVelocity());
  }

  public void aimForWooferSlam() {
    runLeftMotor(leftShooterWooferSlamVelocity);
    runRightMotor(rightShooterWooferSlamVelocity);
    setShooterPosition(shooterAngleWooferSlam);
  }

  public void aimForChuckLow() {
    runLeftMotor(leftShooterChuckLowVelocity);
    runRightMotor(rightShooterChuckLowVelocity);
    setShooterPosition(shooterChuckLowRotation);
  }

  public void aimForChuckHigh() {
    runLeftMotor(leftShooterChuckHighVelocity);
    runRightMotor(rightShooterChuckHighVelocity);
    setShooterPosition(shooterChuckHighRotation);
  }

  public void aimForAuto() {
    runLeftMotor(autoShooterLeftVelocity);
    runRightMotor(autoShooterRightVelocity);
    setShooterPosition(autoShooterAngle);
  }

  public void aimToWall() {
    double distance = rc.drivetrain.getState().Pose.getX();
    MMWaypoint wallWaypoint = firingSolution.calcSolution(distance);
    runLeftMotor(wallWaypoint.getLeftVelocity());
    runRightMotor(wallWaypoint.getRightVelocity());
    setShooterPosition(wallWaypoint.getAngle());
  }

  public void calcFiringSolution() {
    double distanceToSpeaker = rc.navigation.getDistanceToSpeaker();
    if (targetAngleSpeaker != null) {
      // distanceToSpeaker += Math.abs(.2*
      // Math.sin(targetAngleSpeaker.getRadians()));
      // distanceToSpeaker = distanceToSpeaker;
    }
    desiredWaypoint = firingSolution.calcSolution(distanceToSpeaker);
  }

  public MMWaypoint calcManualFiringSolution(Pose2d position) {
    Translation2d target = MMField.currentSpeakerPose().getTranslation();
    double distanceToSpeaker = position.getTranslation().minus(target).getNorm();
    return firingSolution.calcSolution(distanceToSpeaker);
  }

  // public void calcPredictedFiringSolution() {
  // distanceToSpeaker = rc.navigation.getPredictedDistanceToSpeaker();
  // desiredWaypoint = firingSolution.calcSolution(distanceToSpeaker);
  // }

  public Shooter setIntakeFlag(boolean run) {
    runIntake = run;
    return this;
  }

  public Shooter setColorFlag(boolean run) {
    return this;
  }

  public Shooter setReverseIntakeFlag(boolean run) {
    runOutTake = run;
    return this;
  }

  public Shooter setShootFlag(boolean run) {
    runShoot = run;
    return this;
  }

  public Shooter setAbortIntakeFlag(boolean isAborted) {
    abortIntake = isAborted;
    return this;
  }

  public Shooter setChuckLowFlag(boolean isChuck) {
    runChuckLow = isChuck;
    if (isChuck) {
      runAim = false;
    }
    return this;
  }

  public Shooter setChuckHighFlag(boolean isChuck) {
    runChuckHigh = isChuck;
    if (isChuck) {
      runAim = false;
    }
    return this;
  }

  public Shooter setWooferSlamFlag(boolean isSlam) {
    runWooferSlam = isSlam;
    if (isSlam) {
      runAim = false;
    }
    return this;
  }

  public Shooter setShootOverrideFlag(boolean isOverride) {
    runShootOverride = isOverride;
    return this;
  }

  public Shooter setAutoStartForceAngle(boolean value) {
    autoStartForceAngle = value;
    return this;
  }

  public Shooter setAimFlag(boolean aim) {
    if (runAim && !aim) {
      stopShooterMotors();
    }
    if (aim) {
      setAimWallFlag(false);
      runChuckLow = false;
    }
    runAim = aim;
    return this;
  }

  public Shooter setAimWallFlag(boolean aim) {
    if (runWallAim && !aim) {
      stopShooterMotors();
    }
    if (aim) {
      setAimFlag(false);
    }
    runWallAim = aim;
    return this;
  }

  public Shooter setElevatorIndexFlag(boolean index) {
    runElevatorIndex = index;
    return this;
  }

  public Shooter setRunDiagnosticFlag(boolean runTest) {
    runDiagnosticTest = runTest;
    return this;
  }

  public boolean getIntakeFlag() {
    return runIntake;
  }

  public boolean getReverseIntakeFlag() {
    return runOutTake;
  }

  public boolean getShootFlag() {
    return runShoot;
  }

  public boolean getAbortIntakeFlag() {
    return abortIntake;
  }

  public boolean getAimFlag() {
    return runAim;
  }

  public boolean getElevatorIndexFlag() {
    return runElevatorIndex;
  }

  public boolean getRunDiagnosticFlag(boolean runTest) {
    return runDiagnosticTest;
  }

  public void runIntakeInSlow() {
    intakeBeltMotor.setControl(intakeBeltVelVol.withVelocity(.4 * intakeVelIn));// .6
  }

  public void runIntakeIn() {
    intakeBeltMotor.setControl(intakeBeltVelVol.withVelocity(intakeVelIn));
  }

  public void runIntakeOut() {
    intakeBeltMotor.setControl(intakeBeltVelVol.withVelocity(intakeVelOut));
  }

  public void stopIntake() {
    intakeBeltMotor.setControl(intakeBeltVelVol.withVelocity(0));
  }

  public void setIntakeUp() {
    intakeRotateMotor.setControl(intakeRotateMotionMagicVoltage.withSlot(0).withPosition(intakeUpPos));
  }

  public void setIntakeDown() {
    intakeRotateMotor.setControl(intakeRotateMotionMagicVoltage.withSlot(0).withPosition(intakeDownPos));
  }

  public void runIndexers(double index1Speed, double index2Speed) {
    index1.setControl(index1VelVol.withVelocity(index1Speed));
    index2.setControl(index2VelVol.withVelocity(index2Speed));
  }

  public void runIndexShoot() {
    index1.setControl(index1VelVol.withVelocity(desiredWaypoint.getLeftVelocity()));
    index2.setControl(index1VelVol.withVelocity(desiredWaypoint.getRightVelocity()));
  }

  public void runIndexIn() {
    index1.setControl(index1VelVol.withVelocity(index1InVel));
    index2.setControl(index2VelVol.withVelocity(index2InVel));
  }

  public void runIndexOut() {
    index1.setControl(index1VelVol.withVelocity(index1OutVel));
    index2.setControl(index2VelVol.withVelocity(index2OutVel));
  }

  public void stopIndexers() {
    index2.set(0);
    index1.set(0);
  }

  public void stopMotors() {
    VoltageOut vv = new VoltageOut(0);
    intakeBeltMotor.setControl(vv);
    intakeRotateMotor.setControl(vv);
    shooterRotateMotor.setControl(vv);
    index1.setControl(vv);
    index2.setControl(vv);
    leftMotor.setControl(vv);
    rightMotor.setControl(vv);
    elevatorMotor.setControl(vv);
  }

  public void resetStateMachine() {
    ssm.setInitial(ssm.Start);
  }

  public void setShotStartTime() {
    shotStartTime = Timer.getFPGATimestamp();
  }

  public boolean readyToShoot() {
    boolean leftShooterAtVelocity = isInMargin(leftMotor.getVelocity().getValue(), desiredWaypoint.getLeftVelocity(),
        shooterVelocityMargin);
    boolean rightShooterAtVelocity = isInMargin(rightMotor.getVelocity().getValue(), desiredWaypoint.getRightVelocity(),
        shooterVelocityMargin);
    boolean shooterAtAngle = isInMargin(shooterRotateMotor.getPosition().getValue(), desiredWaypoint.getAngle(),
        shooterAngleMargin);
    atShooterAngleLog.append(shooterAtAngle);
    atSpeedLog.append(rightShooterAtVelocity && leftShooterAtVelocity);

    // Removed as obsolete
    // boolean atBoundary = ((leftBoundaryAngleSpeaker.getDegrees() <
    // currentPose.getRotation().getDegrees()
    // && rightBoundaryAngleSpeaker.getDegrees() >
    // currentPose.getRotation().getDegrees()) ||
    // (rightBoundaryAngleSpeaker.getDegrees() <
    // currentPose.getRotation().getDegrees()
    // && leftBoundaryAngleSpeaker.getDegrees() >
    // currentPose.getRotation().getDegrees()));

    SmartDashboard.putNumber("leftMotor Actual", leftMotor.getVelocity().getValue());
    SmartDashboard.putNumber("RightMotor Actual", rightMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Shooter Rotate Motor Actual", shooterRotateMotor.getPosition().getValue());
    SmartDashboard.putNumber("desired left motor", desiredWaypoint.getLeftVelocity());
    SmartDashboard.putNumber("desired right motor", desiredWaypoint.getRightVelocity());
    SmartDashboard.putNumber("desired rotate motor", desiredWaypoint.getAngle());
    SmartDashboard.putNumber("desired Pose Something",
        Math.abs(currentPose.getRotation().minus(targetAngleSpeaker).getDegrees()));
    SmartDashboard.putBoolean("at velocity left", leftShooterAtVelocity);
    SmartDashboard.putBoolean("at velocity right", rightShooterAtVelocity);
    SmartDashboard.putBoolean("at Shooter angle", shooterAtAngle);
    // SmartDashboard.putBoolean("at Boundary", atBoundary);
    SmartDashboard.putNumber("angle left boundary", leftBoundaryAngleSpeaker.getDegrees());
    SmartDashboard.putNumber("angle right boundary", rightBoundaryAngleSpeaker.getDegrees());
    SmartDashboard.putNumber("angle Robot", currentPose.getRotation().getDegrees());

    // Try using the Pose and a transform to project your position forward by your
    // distance to the target.
    // If the resulting point (translation) is within the width of the speaker,
    // bingo!
    // Review the below code and clean it up to make it work.
    // Don't get locked into the code below, but start by understanding it and
    // making it work.

    // TODO: TRY 360 NO-Scope again, but reverse b and d...
    // a.plus(b).plus(d), this should be our real position, modified by our
    // velocity, modified by the vector to the speaker.
    // double shotTime = 5;
    // ChassisSpeeds chassisSpeeds = rc.drivetrain.getCurrentRobotChassisSpeeds();
    // Pose2d a = currentPose;
    // Transform2d inTargetTransform = new Transform2d(new
    // Translation2d(rc.navigation.getDistanceToSpeaker(), 0),
    // new Rotation2d());
    // Transform2d predictedTransform = new Transform2d(new
    // Translation2d(chassisSpeeds.vxMetersPerSecond * shotTime,
    // chassisSpeeds.vyMetersPerSecond * shotTime),
    // new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * shotTime));
    // Translation2d c =
    // MMField.getBlueTranslation(a.plus(predictedTransform).plus(inTargetTransform).getTranslation());

    // leadGuessPosePublisher.set(a.plus(predictedTransform));

    // leadActualPosition.append(a.toString());
    // leadXVelocity.append(chassisSpeeds.vxMetersPerSecond);
    // leadYVelocity.append(chassisSpeeds.vyMetersPerSecond);
    // leadARotation.append(chassisSpeeds.omegaRadiansPerSecond);
    // leadGuessPosition.append(a.plus(predictedTransform).toString());

    Pose2d a = currentPose;
    currentPosePublisher.set(a);
    Transform2d b = new Transform2d(new Translation2d(rc.navigation.getDistanceToSpeaker(), 0), new Rotation2d());
    Translation2d c = MMField.getBlueTranslation(a.plus(b).getTranslation());

    // TODO: Reveiw the following change to bingo to handle NOT resetting angle to
    // something custom.
    boolean bingo = isInMargin(c.getY(),
        MMField.blueSpeakerPose.getTranslation().getY(), .3)// .3556
        && Math.abs(Robot.allianceSpeakerRotation.minus(currentPose.getRotation()).getDegrees()) < 90;
    SmartDashboard.putBoolean("bingo", bingo);
    atTargetAngleLog.append(bingo);

    return leftShooterAtVelocity
        && rightShooterAtVelocity
        && shooterAtAngle
        && (bingo || autoStartForceAngle);
  }

  public boolean readyToChuck() {
    return isInMargin(getLeftShooterVelocity(), leftShooterChuckLowVelocity, shooterVelocityMargin) &&
        isInMargin(getRightShooterVelocity(), rightShooterChuckLowVelocity, shooterVelocityMargin) &&
        isInMargin(getShooterAngle(), shooterChuckLowRotation, shooterAngleMargin);
  }

  public boolean readyToChuckHigh() {
    return isInMargin(getLeftShooterVelocity(), leftShooterChuckHighVelocity, shooterVelocityMargin) &&
        isInMargin(getRightShooterVelocity(), rightShooterChuckHighVelocity, shooterVelocityMargin) &&
        isInMargin(getShooterAngle(), shooterChuckHighRotation, shooterAngleMargin);
  }

  public boolean readyToWooferSlam() {
    return isInMargin(getLeftShooterVelocity(), leftShooterWooferSlamVelocity, shooterVelocityMargin) &&
        isInMargin(getRightShooterVelocity(), rightShooterWooferSlamVelocity, shooterVelocityMargin) &&
        isInMargin(getShooterAngle(), shooterAngleWooferSlam, shooterAngleMargin);
  }

  public boolean isInMargin(double value1, double value2, double margin) {
    return Math.abs(value1 - value2) <= margin;
  }

  public String currentStateName() {
    return ssm.currentState.getName();
  }

  public void configIntakeBeltMotor() {
    TalonFXConfiguration genericConfig = new TalonFXConfiguration();
    genericConfig.Slot0
        .withKS(.25)
        .withKV(.12)
        .withKA(.01)
        .withKG(0)
        .withKP(.125)
        .withKI(0)
        .withKD(0);
    // MMConfigure.configureDevice(leftMotor, genericConfig);

    genericConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    MMConfigure.configureDevice(elevatorBottomBelt, genericConfig);
    MMConfigure.configureDevice(elevatorTopBelt, genericConfig);

    // MMConfigure.configureDevice(rightMotor, genericConfig);
    genericConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    MMConfigure.configureDevice(intakeBeltMotor, genericConfig);

  }

  public void configShooterMotors() {
    TalonFXConfiguration genericConfig = new TalonFXConfiguration();
    genericConfig.Slot0
        .withKS(.25)
        .withKV(.12)
        .withKA(.01)
        .withKG(0)
        .withKP(.125)
        .withKI(0)
        .withKD(0);
    genericConfig.MotionMagic
        .withMotionMagicAcceleration(100 / 1)
        .withMotionMagicJerk((100 / 1) / .2);
    MMConfigure.configureDevice(leftMotor, genericConfig);

    genericConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    MMConfigure.configureDevice(rightMotor, genericConfig);
    // MMConfigure.configureDevice(intakeBeltMotor, genericConfig);

  }

  public void configIndexMotors() {
    TalonFXConfiguration genericConfig = new TalonFXConfiguration();
    genericConfig.Slot0
        .withKS(.25)
        .withKV(.12)
        .withKA(.01)
        .withKG(0)
        .withKP(.125)
        .withKG(0)
        .withKI(0)
        .withKD(0);
    genericConfig.MotorOutput
        .withNeutralMode(NeutralModeValue.Brake);

    MMConfigure.configureDevice(index1, genericConfig);

    genericConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    MMConfigure.configureDevice(index2, genericConfig);

  }

  private void configShooterRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0.1904296875);
    MMConfigure.configureDevice(shooterRotateCanCoder, canConfig);
  }

  private void configIntakeRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.165);
    MMConfigure.configureDevice(intakeRotateCanCoder, canConfig);
  }

  private void configIntakeRotateMotor() {
    double cruiseVelocity = 4; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //

    double sensorLow = 0.04;
    double sensorHigh = 0.84;
    double rotorLow = -9.8;
    double rotorHigh = -0.6;
    double calcRotorToSensor = (rotorHigh - rotorLow) / (sensorHigh - sensorLow);

    double rotorToSensorRatio = calcRotorToSensor;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.CurrentLimits.SupplyCurrentLimit = 4;
    cfg.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(0) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(16)
        .withKI(0)
        .withKD(0);
    cfg.Feedback
        .withFeedbackRemoteSensorID(intakeRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    MMConfigure.configureDevice(intakeRotateMotor, cfg);
  }

  private void configShooterRotateMotor() {
    double cruiseVelocity = .5; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .2; // seconds
    double timeToReachMaxAcceleration = .1; // seconds

    double sensorLow = 0.383;
    double sensorHigh = .455;
    double rotorLow = -34.7;
    double rotorHigh = -4.91;
    double calcRotorToSensor = (rotorHigh - rotorLow) / (sensorHigh - sensorLow);

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = calcRotorToSensor;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive)
        .withDutyCycleNeutralDeadband(.05);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(0) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(.5) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(330)// 330
        .withKI(0)
        .withKD(3.3);
    cfg.Feedback
        .withFeedbackRemoteSensorID(shooterRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    MMConfigure.configureDevice(shooterRotateMotor, cfg);
  }

  private void configElevatorMotor() {
    double cruiseVelocity = 80;
    double timeToReachCruiseVelocity = .125;
    double timeToReachMaxAcceleration = .05;

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = 1;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max

    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(0) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(.1) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(.4)
        .withKI(0)
        .withKD(0);
    MMConfigure.configureDevice(elevatorMotor, cfg);
  }

  public void startOrchestra() {
    // orchestra.play();
  }

  public void stopOrchestra() {
    // orchestra.stop();
  }

  public void setElevatorUp() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorAmpPosition));
  }

  public void setElevatorUpTrap() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorTrapPosition));
  }

  public void setElevatorUpTrapShot() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorTrapShootPosition));
  }

  public void setElevatorDown() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorDownPosition));
  }

  public void runElevatorToHome() {
    elevatorMotor.setControl(elevatorVoltageOut.withOutput(-.5));
  }

  public void setElevatorZero() {
    elevatorMotor.setPosition(elevatorDownPosition);
  }

  public void runElevatorBottomBeltUpFast() {
    elevatorBottomBelt.setControl(elevatorVelVol.withVelocity(intakeVelOut * 3));
  }

  public void runElevatorTopBeltUpFast() {
    elevatorTopBelt.setControl(elevatorVelVol.withVelocity(intakeVelOut * 3));
  }

  public void runElevatorBottomBeltDownFast() {
    elevatorBottomBelt.setControl(elevatorVelVol.withVelocity(intakeVelIn * 3));
  }

  public void runElevatorTopBeltDownFast() {
    elevatorTopBelt.setControl(elevatorVelVol.withVelocity(intakeVelIn * 3));
  }

  public void runElevatorBottomBeltUpSlow() {
    elevatorBottomBelt.setControl(elevatorVelVol.withVelocity(-20));
  }

  public void runElevatorTopBeltUpSlow() {
    elevatorTopBelt.setControl(elevatorVelVol.withVelocity(-20));
  }

  public void runElevatorBottomBeltDownSlow() {
    elevatorBottomBelt.setControl(elevatorVelVol.withVelocity(20));
  }

  public void runElevatorTopBeltDownSlow() {
    elevatorTopBelt.setControl(elevatorVelVol.withVelocity(20));
  }

  public void runElevatorBeltShoot() {
    elevatorTopBelt.setControl(elevatorVelVol.withVelocity(60));
  }

  public void stopElevator() {
    elevatorMotor.setControl(elevatorVoltageOut.withOutput(0));
  }

  public void stopElevatorBelts() {
    elevatorBottomBelt.setControl(elevatorVoltageOut.withOutput(0));
    elevatorTopBelt.setControl(elevatorVoltageOut.withOutput(0));
  }

  public void logUpdate() {

    shooterStateLog.append(ssm.currentState.getName());
    currentPosePublisher.accept(currentPose);
    intakeFlagLog.append(runIntake);
    aimFlagLog.append(runAim);
    intakePositionLog.append(getIntakePos());
    intakeVelocityLog.append(getIntakeVel());
    leftShooterVelocityLog.append(getLeftShooterVelocity());
    rightShooterVelocityLog.append(getRightShooterVelocity());
    rightIndexVelocityLog.append(index1.getVelocity().getValue());
    shooterRotatePositionLog.append(getShooterAngle());
    leftIndexVelocityLog.append(index2.getVelocity().getValue());
    intakeBreakBeamLog.append(intakeBreakBeam.get());
    shooterBreakBeamLog.append(shooterBreakBeam.get());

  }

  public void shooterDown() {
    setShooterPosition(.385);
  }

  public boolean getShooterBreakBeam() {
    return shooterBreakBeam.get();
  }

  public void setReadyToAutoShoot(boolean isTrue) {
    shootAutoShot = isTrue;
  }

  public void setAutoForce(boolean isTrue) {
    autoForceShot = isTrue;
  }

  public boolean isReadyToAutoShoot() {
    boolean leftShooterAtVelocity = isInMargin(leftMotor.getVelocity().getValue(), autoShooterLeftVelocity,
        shooterVelocityMargin);
    boolean rightShooterAtVelocity = isInMargin(rightMotor.getVelocity().getValue(), autoShooterRightVelocity,
        shooterVelocityMargin);
    boolean shooterAtAngle = isInMargin(shooterRotateMotor.getPosition().getValue(), autoShooterAngle,
        shooterAngleMargin);
    Pose2d a = currentPose;
    currentPosePublisher.set(a);
    Transform2d b = new Transform2d(new Translation2d(rc.navigation.getDistanceToSpeaker(), 0), new Rotation2d());
    Translation2d c = MMField.getBlueTranslation(a.plus(b).getTranslation());
    boolean bingo = isInMargin(c.getY(),
        MMField.blueSpeakerPose.getTranslation().getY(), .3)// .3556
        && Math.abs(Robot.allianceSpeakerRotation.minus(currentPose.getRotation()).getDegrees()) < 90;
    SmartDashboard.putBoolean("AutoleftShooterAt", leftShooterAtVelocity);
    SmartDashboard.putBoolean("AutorightShooterAt", rightShooterAtVelocity);
    SmartDashboard.putBoolean("AutoshooterAt", shooterAtAngle);
    SmartDashboard.putBoolean("Autobingo", bingo);

    return leftShooterAtVelocity && rightShooterAtVelocity && shooterAtAngle && bingo;
  }

  public void setLeftAutoShooterVelocity(double v) {
    autoShooterLeftVelocity = v;
  }

  public void getRidOfNote() {
    runLeftMotor(leftShooterRidLowVelocity);
    runRightMotor(rightShooterRidLowVelocity);
  }

  public void setRightAutoShooterVelocity(double v) {
    autoShooterRightVelocity = v;

  }

  public void setAutoShooterAngle(double v) {
    autoShooterAngle = v + autoShotChange;

  }

  public void populateAutoShot(String shotName) {
    autoShooterLeftVelocity = determineShot.get(shotName).getLeftVelocity();
    autoShooterRightVelocity = determineShot.get(shotName).getRightVelocity();
    autoShooterAngle = determineShot.get(shotName).getAngle();
  }

  // MMStateMachineState DiagnosticRunIndexIn = new
  // MMStateMachineState("DiagnosticRunIndexIn") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runIndexIn();
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredIndexIn = isInMargin(index1InVel, getIndex1Vel(),
  // shooterVelocityMargin)
  // && isInMargin(index2InVel, getIndex2Vel(), shooterVelocityMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredIndexIn) {
  // return DiagnosticRunIndexOut;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };

  // MMStateMachineState DiagnosticRunIndexOut = new
  // MMStateMachineState("DiagnosticRunIndexOut") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runIndexOut();
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredIndexOut = isInMargin(index2OutVel, getIndex2Vel(),
  // shooterVelocityMargin)
  // && isInMargin(index1OutVel, getIndex1Vel(), shooterVelocityMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredIndexOut) {
  // return DiagnosticShooterAngle;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };

  // MMStateMachineState DiagnosticAngle = new
  // MMStateMachineState("DiagnosticAngle") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // stopIndexers();
  // setShooterPosition(diagnosticShooterAngle);
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredShooterAngle = isInMargin(diagnosticShooterAngle,
  // getShooterAngle(), shooterAngleMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredShooterAngle) {
  // return DiagnosticRightShoot;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };

  // MMStateMachineState DiagnosticLeftShoot = new
  // MMStateMachineState("DiagnosticLeftShooter") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runLeftMotor(diagnosticLeftMotorSpeed);
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredLeftShooterVel = isInMargin(diagnosticLeftMotorSpeed,
  // getLeftShooterVelocity(),
  // shooterVelocityMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredLeftShooterVel) {
  // return DiagnosticElevatorUp;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }

  // @Override
  // public void transitionFrom(MMStateMachineState nextState) {
  // stopLeftMotor();
  // }
  // };

  // MMStateMachineState DiagnosticRightShoot = new
  // MMStateMachineState("DiagnosticRightShooter") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runRightMotor(diagnosticRightMotorSpeed);
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredRightShooterVel = isInMargin(diagnosticRightMotorSpeed,
  // getRightShooterVelocity(),
  // shooterVelocityMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredRightShooterVel) {
  // return DiagnosticLeftShoot;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }

  // @Override
  // public void transitionFrom(MMStateMachineState nextState) {
  // stopRightMotor();
  // }

  // };

  // MMStateMachineState DiagnosticElevatorUp = new
  // MMStateMachineState("DiagnosticElevatorUp") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // setElevatorUp();
  // // rc.joystick.setRumble
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredElevatorUp =
  // isInMargin(elevatorMotor.getPosition().getValue(), elevatorAmpPosition,
  // elevatorPositionMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredElevatorUp) {
  // return DiagnosticElevatorDown;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };

  // MMStateMachineState DiagnosticElevatorDown = new
  // MMStateMachineState("DiagnosticElevatorDown") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // setElevatorDown();
  // // rc.joystick.setRumble
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredElevatorDown =
  // isInMargin(elevatorMotor.getPosition().getValue(), elevatorDownPosition,
  // elevatorPositionMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredElevatorDown) {
  // return DiagnosticElevatorBeltUp;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };

  // MMStateMachineState DiagnosticElevatorBeltUp = new
  // MMStateMachineState("DiagnosticElevatorBeltUp") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runElevatorBeltUpSlow();
  // // rc.joystick.setRumble
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredElevatorBeltUp =
  // isInMargin(elevatorBelt.getVelocity().getValue(), DiagnosticElevatorBeltVel,
  // DiagnosticElevatorBeltMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredElevatorBeltUp) {
  // return DiagnosticElevatorBeltDown;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }

  // };

  // MMStateMachineState DiagnosticElevatorBeltDown = new
  // MMStateMachineState("DiagnosticElevatorBeltDown") {
  // @Override
  // public void transitionTo(MMStateMachineState previousState) {
  // runElevatorBeltDownSlow();
  // // rc.joystick.setRumble
  // }

  // @Override
  // public void doState() {
  // diagnosticDesiredElevatorBeltDown =
  // isInMargin(elevatorBelt.getVelocity().getValue(),
  // -DiagnosticElevatorBeltVel, DiagnosticElevatorBeltMargin);
  // }

  // @Override
  // public MMStateMachineState calcNextState() {
  // if (timeInState >= diagnosticRunTime && diagnosticDesiredElevatorBeltDown) {
  // return Idle;
  // }
  // if (timeInState >= diagnosticTimeOut) {
  // return Idle;
  // }
  // return this;
  // }
  // };
}
