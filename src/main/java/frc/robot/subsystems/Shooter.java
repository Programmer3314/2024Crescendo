// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import edu.wpi.first.wpilibj.DigitalInput;
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
  private ShooterStateMachine ssm = new ShooterStateMachine();
  private MMTurnPIDController turnPidController = new MMTurnPIDController();
  Rotation2d targetAngleSpeaker;
  Rotation2d leftBoundaryAngleSpeaker;
  Rotation2d rightBoundaryAngleSpeaker;
  Pose2d speakerPose;
  Pose2d currentPose;
  double speakerTurnRate;
  // TODO: Review the followign margins
  double shooterAngleMargin = .1;
  double shooterVelocityMargin = 20;
  double rotationMargin = 2;

  boolean runAim;
  boolean runIntake;
  boolean abortIntake;
  boolean runShoot;
  boolean runOutTake;
  double shooterDelay = .125;
  double outTakeDelay = .125;

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

  public double distanceToSpeaker;
  public MMWaypoint desiredWaypoint;

  MMFiringSolution firingSolution = new MMFiringSolution(
      new MMWaypoint(1.3, .45, 35, 45, 40),
      new MMWaypoint(2.12, .425, 37, 48, 42),
      new MMWaypoint(3.55, .39, 40, 50, 45));

  private TalonFX intakeBeltMotor = new TalonFX(9, "CANIVORE");
  private TalonFX intakeRotateMotor = new TalonFX(10, "CANIVORE");
  private TalonFX shooterRotateMotor = new TalonFX(11, "CANIVORE");
  private TalonFX index1 = new TalonFX(12, "CANIVORE");
  private TalonFX index2 = new TalonFX(13, "CANIVORE");
  private TalonFX leftMotor = new TalonFX(14, "CANIVORE");
  private TalonFX rightMotor = new TalonFX(15, "CANIVORE");
  private TalonFX elevatorMotor = new TalonFX(16, "CANIVORE");

  CANcoder intakeRotateCanCoder = new CANcoder(5, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(6, "CANIVORE");

  DigitalInput intakeBreakBeam = new DigitalInput(0);// NOTE: broken = false, solid = true
  DigitalInput shooterBreakBeam = new DigitalInput(1);

  double intakeUpPos = .75;
  double intakeDownPos = 0.1;

  double intakeVelIn = 20;
  double intakeVelOut = -intakeVelIn;

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
  private VelocityVoltage rightVelVol = new VelocityVoltage(0);
  private VelocityVoltage intakeBeltVelVol = new VelocityVoltage(0);
  private VoltageOut elevatorvoVoltageOut = new VoltageOut(0);

  /** Creates a new Shooter. */
  public Shooter(RobotContainer rc) {
    this.rc = rc;
    configShooterRotateCanCoder();
    configShooterRotateMotor();
    configIntakeRotateCanCoder();
    configIntakeRotateMotor();
    configMotors();
    configIndexMotors();
    configElevatorMotors();
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
    Translation2d transformLeftBoundarySpeaker = MMField.currentLeftBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    leftBoundaryAngleSpeaker = transformLeftBoundarySpeaker.getAngle();
    Translation2d transformRightBoundarySpeaker = MMField.currentRightBoundaryPose().getTranslation()
        .minus(currentPose.getTranslation());
    rightBoundaryAngleSpeaker = transformRightBoundarySpeaker.getAngle();

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
      public MMStateMachineState calcNextState() {// TODO create sequence that decides which state to go to
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
        // TODO: THINK ABOUT... 
        // Should this actually turn on when the note hits the intake sensor?
        setAimFlag(true);
      }

      @Override
      public MMStateMachineState calcNextState() {
        // TODO: this appears to be (sort of) duplicate of line in Periodic
        SmartDashboard.getBoolean("Shooter Breakbeam", shooterBreakBeam.get());
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
        indexCounter++;
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

      // TODO: HIGH PRIORITY We are missing some stuff, like doing the things needed to shoot.
      // They are probably set already, but maybe we should make sure.
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
        runIntakeIn();
        runIndexShoot();
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
        diagnosticDesiredIntakeDown = isInMargin(intakeDownPos, getIntakePos(), rotationMargin);
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
        diagnosticDesiredIntakeIn = isInMargin(intakeVelIn, getIntakeVel(), shooterVelocityMargin);
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
        diagnosticDesiredIntakeOut = isInMargin(intakeVelOut, getIntakeVel(), shooterVelocityMargin);
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
        diagnosticDesiredIntakeUp = isInMargin(intakeUpPos, getIntakePos(), rotationMargin);
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

  public void setElevatorPosition(double position) {
    elevatorMotor.setControl(elevatorMotorMotionMagicVoltage.withPosition(position));
  }

  public void setElevatorVoltage(double voltage) {
    elevatorMotor.setControl(elevatorvoVoltageOut.withOutput(voltage));
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
    Pose2d a = currentPose;
    Transform2d b = new Transform2d(new Translation2d(rc.navigation.getDistanceToSpeaker(), 0), new Rotation2d());
    Translation2d c = MMField.getBlueTranslation(a.plus(b).getTranslation());
    boolean bingo = isInMargin(c.getY(),
        MMField.blueSpeakerPose.getTranslation().getY(), .3556);

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

  // TODO: Convert Shooter Motors (left & right) to MotionMagicVelocityVoltage.
  // The motors spin up harshly (based on the sound they make).
  // This will require additional configs (.MotionMagic)
  // Split ConfigMotors 
  // Change Shooter motor control requests to MotionMagicVelocityVoltage 
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
    MMConfigure.configureDevice(leftMotor, genericConfig);

    genericConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    MMConfigure.configureDevice(rightMotor, genericConfig);
    MMConfigure.configureDevice(intakeBeltMotor, genericConfig);

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
        .withMagnetOffset(-0.15);
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
        .withInverted(InvertedValue.Clockwise_Positive);
    cfg.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    cfg.Slot0
        .withKS(0) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(1) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(300)
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
        .withKP(0)
        .withKI(0)
        .withKD(0);
    MMConfigure.configureDevice(elevatorMotor, cfg);
  }

}
