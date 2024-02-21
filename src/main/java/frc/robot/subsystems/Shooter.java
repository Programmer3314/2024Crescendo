// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;
import frc.robot.MMUtilities.MMTurnPIDController;
import frc.robot.MMUtilities.MMWaypoint;

public class Shooter extends SubsystemBase {
  RobotContainer rc;

  StringLogEntry leadActualPosition;
  StringLogEntry leadGuessPosition;
  DoubleLogEntry leadXVelocity;
  DoubleLogEntry leadYVelocity;
  DoubleLogEntry leadARotation;

  private ShooterStateMachine ssm = new ShooterStateMachine();
  private MMTurnPIDController turnPidController = new MMTurnPIDController(true);
  Rotation2d targetAngleSpeaker;
  Rotation2d leftBoundaryAngleSpeaker;
  Rotation2d rightBoundaryAngleSpeaker;
  Pose2d speakerPose;
  Pose2d currentPose;
  double speakerTurnRate;
  // TODO: Review the followign margins
  double shooterAngleMargin = .0015;
  double shooterVelocityMargin = 5;
  double intakeVelocityMargin = 20;
  double intakeRotationMargin = .1;

  int abortIntakeCounter;

  boolean runAim;
  boolean runIntake;
  boolean abortIntake;
  boolean runElevatorIndex;
  boolean runElevatorShot;
  boolean runShoot;
  boolean runOutTake;
  double shooterDelay = .125;
  double outTakeDelay = .125;

  double shotStartTime;
  double shotEndTime;
  double shotTotalTime;

  boolean runDiagnosticTest;
  double diagnosticRunTime = 3;
  double diagnosticTimeOut = 5;
  double diagnosticShooterAngle = .41;
  double diagnosticLeftMotorSpeed = 50;
  double diagnosticRightMotorSpeed = diagnosticLeftMotorSpeed;

  boolean diagnosticDesiredIntakeUp;
  boolean diagnosticDesiredIntakeDown;
  boolean diagnosticDesiredIntakeIn;
  boolean diagnosticDesiredIntakeOut;
  boolean diagnosticDesiredIndexIn;
  boolean diagnosticDesiredIndexOut;
  boolean diagnosticDesiredShooterAngle;
  boolean diagnosticDesiredRightShooterVel;
  boolean diagnosticDesiredLeftShooterVel;

  double elevatorHomingVelocity = -20;
  boolean hasHomedElevator;

  public double distanceToSpeaker;
  public MMWaypoint desiredWaypoint;

  MMFiringSolution firingSolution;

  private TalonFX intakeBeltMotor = new TalonFX(9, "CANIVORE");
  private TalonFX intakeRotateMotor = new TalonFX(10, "CANIVORE");
  private TalonFX shooterRotateMotor = new TalonFX(11, "CANIVORE");
  private TalonFX index1 = new TalonFX(12, "CANIVORE");
  private TalonFX index2 = new TalonFX(13, "CANIVORE");
  private TalonFX leftMotor = new TalonFX(14, "CANIVORE");
  private TalonFX rightMotor = new TalonFX(15, "CANIVORE");
  private TalonFX elevatorMotor = new TalonFX(16, "CANIVORE");
  private TalonFX elevatorBelt = new TalonFX(17, "CANIVORE");

  CANcoder intakeRotateCanCoder = new CANcoder(5, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(6, "CANIVORE");

  DigitalInput intakeBreakBeam = new DigitalInput(3);// NOTE: broken = false, solid = true
  DigitalInput shooterBreakBeam = new DigitalInput(2);
  DigitalInput elevatorHomeSensor = new DigitalInput(1);
  DigitalInput elevatorBreakBeam = new DigitalInput(4);

  double intakeTop = .924;

  double elevatorInPerRev = 5.125 / 30;

  double intakeUpPos = intakeTop - .02;
  double intakeDownPos = intakeTop - .80;

  double intakeVelIn = 30;
  double intakeVelOut = -intakeVelIn;

  double elevatorDownPosition = 33;
  double elevatorUpPosition = 63;

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
  private VoltageOut elevatorvoVoltageOut = new VoltageOut(0);
  private VelocityVoltage elevatorVelVol = new VelocityVoltage(0);

  StructPublisher<Pose2d> currentPosePublisher = NetworkTableInstance.getDefault()
      // some code to get the pose
      // out displaying on advantagescope.
      // It's pretty laggy but it works.
      // Should be helpful for getting the
      // lead out code sorted and for after
      // matches.
      .getStructTopic("currentPose", Pose2d.struct).publish();
  StructPublisher<Pose2d> leadGuessPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("leadGuessPose", Pose2d.struct).publish();

  /** Creates a new Shooter. */
  public Shooter(RobotContainer rc) {
    this.rc = rc;
    firingSolution = new MMFiringSolution(
        rc,
        new MMWaypoint(1.3, .45, 35, 45, 40),
        new MMWaypoint(2.12, .425, 40, 50, 45),
        new MMWaypoint(2.81, 0.394, 40, 50, 45),
        new MMWaypoint(3.55, .39, 40, 50, 45));
    configShooterRotateCanCoder();
    configShooterRotateMotor();
    configIntakeRotateCanCoder();
    configIntakeRotateMotor();
    configIntakeBeltMotor();
    configShooterMotors();
    configIndexMotors();
    configElevatorMotors();
    ssm.setInitial(ssm.Start);
    SmartDashboard.putData("Run Diagnostic",
        new InstantCommand(() -> this.setRunDiagnostic(true)));

    DataLog log = DataLogManager.getLog();
    leadActualPosition = new StringLogEntry(log, "/my/lead/actualPosition");
    leadXVelocity = new DoubleLogEntry(log, "/my/lead/xVelocity");
    leadYVelocity = new DoubleLogEntry(log, "/my/lead/yVelocity");
    leadARotation = new DoubleLogEntry(log, "/my/lead/aRotation");
    leadGuessPosition = new StringLogEntry(log, "/my/lead/actualPosition");
  }

  // prereq: flag to check elevatorIndex / elevatorDeliver, both triggered on
  // button press
  // --Indexed
  // DONE IF: elevatorIndexFlag-> ElevatorToElevatorIndex
  // ElevatorToElevatorIndex{
  // 1) Move elevator to home position
  // 2) Run index reverse(slow)
  // 3)Run intake reverse(slow)
  // 4) Run elevator belt(slow)
  // DONE IF: beam is broken && elevatorDeliver -> MoveElevatorToPosition
  // DONE IF: intake flag--> ElevatorToIndex
  // }
  // ElevatorToIndex{
  // Run Elevator belt(slow)
  // Run intake in
  // }
  // MoveElevatorToPosition{Run elevator up until we reach the position}
  // PrepareToDeliver{
  // 1) Run elevator belt up (slow)
  // DONE IF: beam is not broken -> Deliver
  // }
  // Deliver{
  // 1) Run elevator belt out until we reach a certain number of revolutions
  // }
  //
  //

  @Override
  public void periodic() {

    calcFiringSolution();
    speakerPose = MMField.currentSpeakerPose();
    currentPose = rc.drivetrain.getState().Pose;
    Translation2d transformFromSpeaker = speakerPose.getTranslation().minus(currentPose.getTranslation());
    targetAngleSpeaker = transformFromSpeaker.getAngle();
    Translation2d transformLeftBoundarySpeaker = MMField.currentLeftBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    leftBoundaryAngleSpeaker = transformLeftBoundarySpeaker.getAngle();
    Translation2d transformRightBoundarySpeaker = MMField.currentRightBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    rightBoundaryAngleSpeaker = transformRightBoundarySpeaker.getAngle();

    if (intakeRotateMotor.getSupplyCurrent().getValue() > 4) {
      // abortIntake = true;
      abortIntakeCounter++;
    }

    // if (!hasHomedElevator) {TODO: Uncomment
    // runElevatorToHome();
    // if (elevatorHomeSensor.get()) {
    // elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(0));
    // elevatorMotor.setPosition(0);
    // hasHomedElevator = true;
    // }
    // }

    ssm.update();

    turnPidController.initialize(targetAngleSpeaker);
    speakerTurnRate = turnPidController.execute(currentPose.getRotation());

    if (runAim) {
      aimToSpeaker();
    }

    // else {
    // // stopShooterMotors();
    // }
    if (runDiagnosticTest) {
      SmartDashboard.putBoolean("diagnosticIntakeDown", diagnosticDesiredIntakeDown);
      SmartDashboard.putBoolean("diagnosticIntakeIn", diagnosticDesiredIntakeIn);
      SmartDashboard.putBoolean("diagnosticIntakeOut", diagnosticDesiredIntakeOut);
      SmartDashboard.putBoolean("diagnosticIntakeUp", diagnosticDesiredIntakeUp);
      SmartDashboard.putBoolean("diagnosticIndexIn", diagnosticDesiredIndexIn);
      SmartDashboard.putBoolean("diagnosticIndexOut", diagnosticDesiredIndexOut);
      SmartDashboard.putBoolean("diagnosticShooterAngle", diagnosticDesiredShooterAngle);
      SmartDashboard.putBoolean("diagnosticShooterVel", diagnosticDesiredRightShooterVel);
      SmartDashboard.putBoolean("diagnosticShooterVel", diagnosticDesiredLeftShooterVel);
    }
    SmartDashboard.putBoolean("Shooter Beam", shooterBreakBeam.get());
    SmartDashboard.putBoolean("Intake Beam", intakeBreakBeam.get());
    SmartDashboard.putString("State Machine State", currentStateName());
    SmartDashboard.putBoolean("AbortIntakeFlag", abortIntake);
    SmartDashboard.putNumber("AbortIntakeCounter", abortIntakeCounter);
    SmartDashboard.putNumber("TotalShotTime", shotTotalTime);
  }

  // (Proposed Outline)
  // States:
  // Start; Travel; Extend; Load Note; Reverse; Aim; Spin Up; Shoot
  // Start-Same
  // Travel- Shooter would be down, shooter & feeder wheels would not be moving
  // Extended- Mostly for testing, for any reason we may need it up
  // Load Note- Move the note into a good position for the feeder wheels from the
  // intake
  // Reverse- Test sequence, outtake(something is stuck), just spin the shooter
  // and feeder wheels back
  // Aim - Targeting on the fly, based on the passed in waypoint, wherever we are
  // on the field, the shooter would target the goal.(Only adjusting angle, not
  // moving shooters/feeders)
  // Spin up- For getting ready to shoot, would spin up the shoot motors to
  // desired angle, would also continue targeting like aim.
  // Shoot- Would(in transition to) shoot the note based on the waypoint, by just
  // running the feeder wheels

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
        return Idle;
      };
    };

    MMStateMachineState Idle = new MMStateMachineState("Idle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setShootFlag(false);
        setIntakeUp();
        setAimFlag(false);
        stopIndexers();
        stopShooterMotors();
        stopIntake();
        setRunDiagnostic(false);
        abortIntake = false;
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
        return this;
      }
    };

    MMStateMachineState DropIntake = new MMStateMachineState("Intake") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();
        runIntakeIn();
        // runIndexIn();
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
        runIntakeIn();
        setAimFlag(true);
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
      }

      @Override
      public void transitionFrom(MMStateMachineState nexState) {
        stopIntake();
        setIntakeUp();
      }
    };

    MMStateMachineState Index = new MMStateMachineState("Index") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIndexers();
        setIntakeFlag(false);
        setAimFlag(true);
        indexCounter++;
      }

      @Override
      public MMStateMachineState calcNextState() {
        // if (runElevatorIndex) {
        // return ElevatorIndex;
        // }
        if (runShoot) {
          return PrepareToShoot;
        }
        if (runOutTake) {
          return IntakeReverse;
        }
        return this;
      }
    };

    MMStateMachineState PrepareToShoot = new MMStateMachineState("PrepareToShoot") {

      @Override
      public MMStateMachineState calcNextState() {
        if (readyToShoot() && runShoot) {
          return Shoot;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorIndex = new MMStateMachineState("ElevatorIndex") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBeltUpFast();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!elevatorBreakBeam.get()) {
          return ElevatorWaitForAction;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorWaitForAction = new MMStateMachineState("ElevatorWaitForAction") {
      @Override
      public MMStateMachineState calcNextState() {
        if (runElevatorShot) {
          return SetElevatorHeight;
        }
        return this;
      }
    };

    MMStateMachineState SetElevatorHeight = new MMStateMachineState("SetElevatorHeight") {
      // Pass through state, just to
      // set position, it'll take
      // too long to move the
      // elevator and wait before
      // sending the note up. Maybe that'll change later with a tuned elevator

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setElevatorUp();
      }

      @Override
      public MMStateMachineState calcNextState() {
        return ElevatorPassNoteAbove;
      }
    };

    MMStateMachineState ElevatorPassNoteAbove = new MMStateMachineState("ElevatorPassNoteAbove") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runElevatorBeltUpSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (elevatorBreakBeam.get()) {
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
        if (elevatorBreakBeam.get()) {
          return ElevatorShoot;
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
      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeDown = isInMargin(intakeDownPos, getIntakePos(), intakeRotationMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIntakeDown) {
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
      }

      @Override
      public void doState() {
        double actualvel = getIntakeVel();
        diagnosticDesiredIntakeIn = isInMargin(intakeVelIn, actualvel, intakeVelocityMargin);
        SmartDashboard.putBoolean("intIn", diagnosticDesiredIntakeIn);
        SmartDashboard.putNumber("intDesired",intakeVelIn);
        SmartDashboard.putNumber("intActual",actualvel);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIntakeIn) {
          return DiagnosticIntakeOut;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;

      }
    };
    MMStateMachineState DiagnosticIntakeOut = new MMStateMachineState("DiagnosticIntakeOut") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIntakeOut();
      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeOut = isInMargin(intakeVelOut, getIntakeVel(), intakeVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIntakeOut) {
          return DiagnosticSetIntakeUp;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;

      }
    };
    MMStateMachineState DiagnosticSetIntakeUp = new MMStateMachineState("DiagnosticSetIntakeUp") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        setIntakeUp();

      }

      @Override
      public void doState() {
        diagnosticDesiredIntakeUp = isInMargin(intakeUpPos, getIntakePos(), intakeRotationMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIntakeUp) {
          return DiagnosticRunIndexIn;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;

      }

    };
    MMStateMachineState DiagnosticRunIndexIn = new MMStateMachineState("DiagnosticRunIndexIn") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexIn();

      }

      @Override
      public void doState() {
        diagnosticDesiredIndexIn = isInMargin(index1InVel, getIndex1Vel(), shooterVelocityMargin)
            && isInMargin(index2InVel, getIndex2Vel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIndexIn) {
          return DiagnosticRunIndexOut;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticRunIndexOut = new MMStateMachineState("DiagnosticRunIndexOut") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexOut();
      }

      @Override
      public void doState() {
        diagnosticDesiredIndexOut = isInMargin(index2OutVel, getIndex2Vel(), shooterVelocityMargin)
            && isInMargin(index1OutVel, getIndex1Vel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredIndexOut) {
          return DiagnosticAngle;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticAngle = new MMStateMachineState("DiagnosticAngle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIndexers();
        setShooterPosition(diagnosticShooterAngle);
      }

      @Override
      public void doState() {
        diagnosticDesiredShooterAngle = isInMargin(diagnosticShooterAngle, getShooterAngle(), shooterAngleMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredShooterAngle) {
          return DiagnosticRightShoot;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;

      }
    };

    MMStateMachineState DiagnosticLeftShoot = new MMStateMachineState("DiagnosticLeftShooter") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runLeftMotor(diagnosticLeftMotorSpeed);
      }

      @Override
      public void doState() {
        diagnosticDesiredLeftShooterVel = isInMargin(diagnosticLeftMotorSpeed, getLeftShooterVelocity(),
            shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredLeftShooterVel) {
          return Idle;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
        stopLeftMotor();
      }
    };

    MMStateMachineState DiagnosticRightShoot = new MMStateMachineState("DiagnosticRightShooter") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runRightMotor(diagnosticRightMotorSpeed);
      }

      @Override
      public void doState() {
        diagnosticDesiredRightShooterVel = isInMargin(diagnosticRightMotorSpeed, getRightShooterVelocity(),
            shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime && diagnosticDesiredRightShooterVel) {
          return DiagnosticLeftShoot;
        }
        if (timeInState >= diagnosticTimeOut) {
          return Idle;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
        stopRightMotor();
      }

    };

    MMStateMachineState IntakeStallPause = new MMStateMachineState("IntakeStallPause") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        stopIntake();
        setIntakeUp();
        // rc.joystick.setRumble
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

  public void calcFiringSolution() {
    distanceToSpeaker = rc.navigation.getDistanceToSpeaker();
    desiredWaypoint = firingSolution.calcSolution(distanceToSpeaker);
  }

  public Shooter setIntakeFlag(boolean run) {
    runIntake = run;
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

  public Shooter setAimFlag(boolean aim) {
    if (runAim && !aim) {
      stopShooterMotors();
    }
    runAim = aim;
    return this;
  }

  public Shooter setElevatorIndexFlag(boolean index) {
    runElevatorIndex = index;
    return this;
  }

  public void setRunDiagnostic(boolean isTrue) {
    runDiagnosticTest = isTrue;
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
    boolean atBoundary = ((leftBoundaryAngleSpeaker.getDegrees() < currentPose.getRotation().getDegrees()
        && rightBoundaryAngleSpeaker.getDegrees() > currentPose.getRotation().getDegrees()) ||
        (rightBoundaryAngleSpeaker.getDegrees() < currentPose.getRotation().getDegrees()
            && leftBoundaryAngleSpeaker.getDegrees() > currentPose.getRotation().getDegrees()));
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
    SmartDashboard.putBoolean("at Boundary", atBoundary);
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


    // double shotTime = 5;
    // ChassisSpeeds chassisSpeeds = rc.drivetrain.getCurrentRobotChassisSpeeds();
    // Pose2d a = currentPose;
    // Transform2d d = new Transform2d(new Translation2d(rc.navigation.getDistanceToSpeaker(), 0), new Rotation2d());
    // Transform2d b = new Transform2d(new Translation2d(chassisSpeeds.vxMetersPerSecond * shotTime,
    //     chassisSpeeds.vyMetersPerSecond * shotTime),
    //     new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * shotTime));
    // Translation2d c = MMField.getBlueTranslation(a.plus(d).plus(b).getTranslation());

    // leadGuessPosePublisher.set(a.plus(b));
    // currentPosePublisher.set(a);

    // leadActualPosition.append(a.toString());
    // leadXVelocity.append(chassisSpeeds.vxMetersPerSecond);
    // leadYVelocity.append(chassisSpeeds.vyMetersPerSecond);
    // leadARotation.append(chassisSpeeds.omegaRadiansPerSecond);
    // leadGuessPosition.append(a.plus(b).toString());

    Pose2d a = currentPose;
    Transform2d b = new Transform2d(new
    Translation2d(rc.navigation.getDistanceToSpeaker(), 0), new Rotation2d());
    Translation2d c = MMField.getBlueTranslation(a.plus(b).getTranslation());

    boolean bingo = isInMargin(c.getY(),
        MMField.blueSpeakerPose.getTranslation().getY(), .3556)
        && isInMargin(targetAngleSpeaker.getDegrees(), currentPose.getRotation().getDegrees(), 40);

    SmartDashboard.putBoolean("bingo", bingo);

    return leftShooterAtVelocity
        && rightShooterAtVelocity
        && shooterAtAngle
        && bingo;
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
    // MMConfigure.configureDevice(rightMotor, genericConfig);
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
    genericConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);

    MMConfigure.configureDevice(index1, genericConfig);

    genericConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    MMConfigure.configureDevice(index2, genericConfig);

  }

  private void configIntakeRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.5);
    MMConfigure.configureDevice(intakeRotateCanCoder, canConfig);
  }

  private void configShooterRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0.1904296875);
    MMConfigure.configureDevice(shooterRotateCanCoder, canConfig);
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
    double cruiseVelocity = .25; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

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
        .withKP(320)
        .withKI(0)
        .withKD(3);
    cfg.Feedback
        .withFeedbackRemoteSensorID(shooterRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    MMConfigure.configureDevice(shooterRotateMotor, cfg);
  }

  private void configElevatorMotors() {
    double cruiseVelocity = 20;
    double timeToReachCruiseVelocity = .5;
    double timeToReachMaxAcceleration = .2;

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
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(.1)
        .withKI(0)
        .withKD(0);
    MMConfigure.configureDevice(elevatorMotor, cfg);
  }

  public void setElevatorUp() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorUpPosition));
  }

  public void setElevatorDown() {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(elevatorDownPosition));
  }

  public void runElevatorToHome() {
    elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(-.5));
  }

  public void runElevatorBeltUpFast() {
    elevatorMotor.setControl(elevatorVelVol.withVelocity(20));
  }

  public void runElevatorBeltDownFast() {
    elevatorMotor.setControl(elevatorVelVol.withVelocity(-20));
  }

  public void runElevatorBeltUpSlow() {
    elevatorMotor.setControl(elevatorVelVol.withVelocity(10));
  }

  public void runElevatorBeltDownSlow() {
    elevatorMotor.setControl(elevatorVelVol.withVelocity(-10));
  }

  public void runElevatorBeltShoot() {
    elevatorMotor.setControl(elevatorVelVol.withVelocity(-60));
  }

  public void stopElevator() {
    elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(0));
  }

  public void stopElevatorBelt() {
    elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(0));
  }
}
