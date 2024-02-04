// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.html.HTMLDocument.HTMLReader.IsindexAction;

import org.ejml.equation.Sequence;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  private ShooterStateMachine ssm = new ShooterStateMachine();
  private MMTurnPIDController turnPidController = new MMTurnPIDController();
  Rotation2d targetAngleSpeaker;
  Pose2d speakerPose;
  Pose2d currentPose;
  double speakerTurnRate;
  double shooterAngleMargin = .01;
  double shooterVelocityMargin = 2;
  double rotationMargin = 2;
  // TODO: replace with real margins
  boolean runAim;
  boolean runIntake;
  boolean abortIntake;
  boolean runShoot;
  boolean runOutTake;
  double shooterDelay = .125;
  double outTakeDelay = .125;

  boolean runDiagnosticTest;
  double diagnosticRunTime = 3;
  double diagnosticShooterAngle = .02;
  double diagnosticLeftMotorSpeed = 50;
  double diagnosticRightMotorSpeed = -diagnosticLeftMotorSpeed;;

  boolean diagnosticDesiredIntakeUp;
  boolean diagnosticDesiredIntakeDown;
  boolean diagnosticDesiredIntakeIn;
  boolean diagnosticDesiredIntakeOut;
  boolean diagnosticDesiredIndexIn;
  boolean diagnosticDesiredIndexOut;
  boolean diagnosticDesiredShooterAngle;
  boolean diagnosticDesiredShooterVel;

  public double distanceToSpeaker;
  public MMWaypoint desiredWaypoint;

  MMFiringSolution firingSolution = new MMFiringSolution(
      new MMWaypoint(1.3, -.111, 50),
      new MMWaypoint(2.5, -.078, 70),
      new MMWaypoint(3.97, -.057, 70));

  private TalonFX intakeBelt = new TalonFX(9, "CANIVORE");
  private TalonFX intakeRotateMotor = new TalonFX(10, "CANIVORE");
  private TalonFX shooterRotateMotor = new TalonFX(11, "CANIVORE");
  private TalonFX index1 = new TalonFX(12, "CANIVORE");
  private TalonFX index2 = new TalonFX(13, "CANIVORE");
  private TalonFX leftMotor = new TalonFX(14, "CANIVORE");
  private TalonFX rightMotor = new TalonFX(15, "CANIVORE");

  CANcoder intakeRotateCanCoder = new CANcoder(5, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(6, "CANIVORE");

  DigitalInput intakeBreakBeam = new DigitalInput(0);// TODO broken = false, solid = true
  DigitalInput shooterBreakBeam = new DigitalInput(1);

  double intakeUpPos = .01;
  double intakeDownPos = 0;

  double intakeVelIn = 100;
  double intakeVelOut = -intakeVelIn;

  double index1InVel = 30;
  double index2InVel = -index1InVel;
  double index1OutVel = -index1InVel;
  double index2OutVel = -index2InVel;

  double index1ShootVel = 70;
  double index2ShootVel = -index1ShootVel;
  // TODO Use waypoint Values for shooter index velocity (also change reverse)
  double index1ReverseVel = -index1ShootVel;
  double index2ReverseVel = -index2ShootVel;

  private final MotionMagicVoltage shooterRotateMotionMagicVoltage = new MotionMagicVoltage(0);
  private final MotionMagicVoltage intakeRotateMotionMagicVoltage = new MotionMagicVoltage(0);
  private VelocityVoltage index1VelVol = new VelocityVoltage(0);
  private VelocityVoltage index2VelVol = new VelocityVoltage(0);
  private VelocityVoltage leftVelVol = new VelocityVoltage(0);
  private VelocityVoltage rightVelVol = new VelocityVoltage(0);
  private VelocityVoltage intakeBeltVelVol = new VelocityVoltage(0);

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterRotateCanCoder();
    configShooterRotateMotor();
    configIntakeRotateCanCoder();
    configIntakeRotateMotor();
    configMotors();
    ssm.setInitial(ssm.Start);
    SmartDashboard.putData("Run Diagnostic",
        new InstantCommand(() -> this.setRunDiagnostic(true)));
  }

  @Override
  public void periodic() {
    calcFiringSolution();
    speakerPose = MMField.currentSpeakerPose();
    currentPose = rc.drivetrain.getState().Pose;
    Translation2d transformFromSpeaker = speakerPose.getTranslation().minus(currentPose.getTranslation());
    targetAngleSpeaker = transformFromSpeaker.getAngle();

    ssm.update();

    turnPidController.initialize(targetAngleSpeaker);
    speakerTurnRate = turnPidController.execute(currentPose.getRotation());

    if (runAim) {
      aimToSpeaker();
    } else {
      stopShooterMotors();
    }
    if (runDiagnosticTest) {
      SmartDashboard.putBoolean("diagnosticIntakeDown", diagnosticDesiredIntakeDown);
      SmartDashboard.putBoolean("diagnosticIntakeIn", diagnosticDesiredIntakeIn);
      SmartDashboard.putBoolean("diagnosticIntakeOut", diagnosticDesiredIntakeOut);
      SmartDashboard.putBoolean("diagnosticIntakeUp", diagnosticDesiredIntakeUp);
      SmartDashboard.putBoolean("diagnosticIndexIn", diagnosticDesiredIndexIn);
      SmartDashboard.putBoolean("diagnosticIndexOut", diagnosticDesiredIndexOut);
      SmartDashboard.putBoolean("diagnosticShooterAngle", diagnosticDesiredShooterAngle);
      SmartDashboard.putBoolean("diagnosticShooterVel", diagnosticDesiredShooterVel);
    }
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
      public void transitionFrom(MMStateMachineState nextState) {
      }

      @Override
      public MMStateMachineState calcNextState() {
        return Idle;
      };
    };

    MMStateMachineState Idle = new MMStateMachineState("Idle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeUp();
        setAimFlag(false);
        stopIndexers();
        stopShooterMotors();
        stopIntake();
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
    MMStateMachineState DropIntake = new MMStateMachineState("DropIntake") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setIntakeDown();
        runIntakeIn();
        index1.setControl(index1VelVol.withVelocity(index1InVel));
        index2.setControl(index2VelVol.withVelocity(index2InVel));
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
      }

      @Override
      public MMStateMachineState calcNextState() {
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

    MMStateMachineState Shoot = new MMStateMachineState("Shoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        index1.setControl(index1VelVol.withVelocity(index1ShootVel));
        index2.setControl(index2VelVol.withVelocity(index2ShootVel));
        setShootFlag(false);
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
      public MMStateMachineState calcNextState() {
        if (timeInState >= shooterDelay) {
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
        runIndexers(index1ShootVel, index2ShootVel);
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
        diagnosticDesiredIntakeDown = isInMargin(intakeDownPos, getIntakePos(), rotationMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticIntakeIn;
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
        diagnosticDesiredIntakeIn = isInMargin(intakeVelIn, getIntakeVel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticIntakeOut;
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
        diagnosticDesiredIntakeOut = isInMargin(intakeVelOut, getIntakeVel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticSetIntakeUp;
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
        diagnosticDesiredIntakeUp = isInMargin(intakeUpPos, getIntakePos(), rotationMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticRunIndexIn;
        }
        return this;

      }

    };
    MMStateMachineState DiagnosticRunIndexIn = new MMStateMachineState("DiagnosticRunIndexIn") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexers(index1InVel, index2InVel);

      }

      @Override
      public void doState() {
        diagnosticDesiredIndexIn = isInMargin(index1InVel, getIndex1Vel(), shooterVelocityMargin)
            && isInMargin(index2InVel, getIndex2Vel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticRunIndexOut;
        }
        return this;
      }
    };
    MMStateMachineState DiagnosticRunIndexOut = new MMStateMachineState("DiagnosticRunIndexOut") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runIndexers(index1OutVel, index2OutVel);

      }

      @Override
      public void doState() {
        diagnosticDesiredIndexOut = isInMargin(index2OutVel, getIndex2Vel(), shooterVelocityMargin)
            && isInMargin(index1OutVel, getIndex1Vel(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticAngle;
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
        if (timeInState >= diagnosticRunTime) {
          return DiagnosticShoot;
        }
        return this;

      }

    };
    MMStateMachineState DiagnosticShoot = new MMStateMachineState("DiagnosticShoot") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runMotors(diagnosticLeftMotorSpeed, diagnosticRightMotorSpeed);
      }

      @Override
      public void doState() {
        diagnosticDesiredShooterVel = isInMargin(diagnosticLeftMotorSpeed, getLeftShooterVelocity(),
            shooterVelocityMargin)
            && isInMargin(diagnosticRightMotorSpeed, getRightShooterVelocity(), shooterVelocityMargin);
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= diagnosticRunTime) {
          return Idle;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
        setRunDiagnostic(false);
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
    return intakeRotateMotor.getVelocity().getValue();
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

  public void runMotors(double leftMotorSpeed, double rightMotorSpeed) {
    leftMotor.setControl(leftVelVol.withVelocity(leftMotorSpeed));
    rightMotor.setControl(rightVelVol.withVelocity(rightMotorSpeed));
  }

  public void runLeftMotor(double leftMotorSpeed) {
    leftMotor.setControl(leftVelVol.withVelocity(leftMotorSpeed));
  }

  public void runRightMotor(double rightMotorSpeed) {
    rightMotor.setControl(rightVelVol.withVelocity(rightMotorSpeed));
  }

  public void setShooterPosition(double position) {
    shooterRotateMotor.setControl(shooterRotateMotionMagicVoltage.withPosition(position));
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

  public void runIndexers(double index1Speed, double index2Speed) {
    index1.setControl(index1VelVol.withVelocity(index1Speed));
    index2.setControl(index2VelVol.withVelocity(index2Speed));
  }

  public void stopIndexers() {
    index2.set(0);
    index1.set(0);
  }

  public void stopIntake() {
    intakeBelt.setControl(intakeBeltVelVol.withVelocity(0));
  }

  public void runIntakeOut() {
    intakeBelt.setControl(intakeBeltVelVol.withVelocity(intakeVelOut));
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

  public Shooter setShootFlag(boolean run) {
    runShoot = run;
    return this;
  }

  public Shooter setAimFlag(boolean aim) {
    runAim = aim;
    return this;
  }

  public void setRunDiagnostic(boolean isTrue) {
    runDiagnosticTest = isTrue;
  }

  public void runIntakeIn() {
    intakeBelt.setControl(intakeBeltVelVol.withVelocity(intakeVelIn));
  }

  public Shooter setReverseIntakeFlag(boolean run) {
    runOutTake = run;
    return this;
  }

  public void setIntakeUp() {
    intakeRotateMotor.setControl(intakeRotateMotionMagicVoltage.withSlot(0).withPosition(intakeUpPos));
  }

  public void setIntakeDown() {
    intakeRotateMotor.setControl(intakeRotateMotionMagicVoltage.withSlot(1).withPosition(intakeDownPos));

  }

  public boolean readyToShoot() {

    return isInMargin(leftMotor.getVelocity().getValue(), desiredWaypoint.getLeftVelocity(), shooterVelocityMargin)
        // )Math.abs(leftMotor.getVelocity().getValue() -
        // desiredWaypoint.getLeftVelocity()) < shooterVelocityMargin
        && isInMargin(rightMotor.getVelocity().getValue(), desiredWaypoint.getRightVelocity(), shooterVelocityMargin)
        // && Math.abs(rightMotor.getVelocity().getValue() -
        // desiredWaypoint.getRightVelocity()) < shooterVelocityMargin
        && isInMargin(shooterRotateMotor.getPosition().getValue(), desiredWaypoint.getAngle(), shooterAngleMargin)
        // && Math.abs(shooterRotateMotor.getPosition().getValue() -
        // desiredWaypoint.getAngle()) < shooterAngleMargin
        && Math.abs(currentPose.getRotation().minus(targetAngleSpeaker).getDegrees()) <= rotationMargin;
  }

  public boolean isInMargin(double value1, double value2, double margin) {
    return Math.abs(value1 - value2) <= margin;
  }

  public String currentStateName() {
    return ssm.currentState.getName();
  }

  public void configMotors() {
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
    MMConfigure.configureDevice(rightMotor, genericConfig);
    MMConfigure.configureDevice(leftMotor, genericConfig);
    MMConfigure.configureDevice(index1, genericConfig);
    MMConfigure.configureDevice(index2, genericConfig);
  }

  private void configIntakeRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.390869140625); // TODO: Update with real values
    MMConfigure.configureDevice(intakeRotateCanCoder, canConfig);
  }

  private void configShooterRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.390869140625); // TODO: Update with real values
    MMConfigure.configureDevice(shooterRotateCanCoder, canConfig);
  }

  private void configIntakeRotateMotor() {
    double cruiseVelocity = .5; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = (60.0 / 15.0) * 48.0; // TODO: Update with real values
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity
    // TODO: Update with real values
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput
        .withNeutralMode(NeutralModeValue.Brake);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(1) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(12)
        .withKI(0)
        .withKD(2);
    cfg.Feedback
        .withFeedbackRemoteSensorID(intakeRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    cfg.Slot1
        .withKS(1) // voltage to overcome static friction
        .withKV(0)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(96 * 2)// 12
        .withKI(0)
        .withKD(.25);// 2
    cfg.Slot2
        .withKS(1) // voltage to overcome static friction
        .withKV(0)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(48)// 12
        .withKI(0)
        .withKD(.25);// 2
    MMConfigure.configureDevice(intakeRotateMotor, cfg);
  }

  private void configShooterRotateMotor() {
    double cruiseVelocity = .5; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = (60.0 / 15.0) * 48.0; // TODO: Update with real values
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

    // TODO: Update with real values
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    cfg.MotorOutput
        .withNeutralMode(NeutralModeValue.Brake);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(1) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(12)
        .withKI(0)
        .withKD(2);
    cfg.Feedback
        .withFeedbackRemoteSensorID(intakeRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    cfg.Slot1
        .withKS(1) // voltage to overcome static friction
        .withKV(0)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(96 * 2)// 12
        .withKI(0)
        .withKD(.25);// 2
    cfg.Slot2
        .withKS(1) // voltage to overcome static friction
        .withKV(0)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(48)// 12
        .withKI(0)
        .withKD(.25);// 2
    MMConfigure.configureDevice(shooterRotateMotor, cfg);
  }
}
