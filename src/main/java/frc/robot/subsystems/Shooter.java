// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;
import frc.robot.MMUtilities.MMWaypoint;

public class Shooter extends SubsystemBase {
  RobotContainer rc;
  boolean runIntake;
  double shooterDelay = .125;
  boolean runShoot;
  MMFiringSolution firingSolution = new MMFiringSolution(
      new MMWaypoint(1.3, -.111, 50),
      new MMWaypoint(2.5, -.078, 70),
      new MMWaypoint(3.97, -.057, 70));

  private TalonFX LeftMotor = new TalonFX(3, "CANIVORE");
  private TalonFX rightMotor = new TalonFX(4, "CANIVORE");
  private TalonFX index1 = new TalonFX(1, "CANIVORE");
  private TalonFX index2 = new TalonFX(2, "CANIVORE");

  private VelocityVoltage leftVelVol = new VelocityVoltage(0);
  private VelocityVoltage rightVelVol = new VelocityVoltage(0);

  TalonFX intakeRotateMotor = new TalonFX(12, "CANIVORE");
  CANcoder intakeRotateCanCoder = new CANcoder(5, "CANIVORE");

  TalonFX shooterRotateMotor = new TalonFX(13, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(6, "CANIVORE");

  TalonFX intakeBelt = new TalonFX(14, "CANIVORE");

  DigitalInput intakeBreakBeam = new DigitalInput(0);// TODO broken = false, solid = true
  DigitalInput shooterBreakBeam = new DigitalInput(1);

  /** Creates a new Shooter. */
  public Shooter() {
    configShooterRotateCanCoder();
    configShooterRotateMotor();
    configIntakeRotateCanCoder();
    configIntakeRotateMotor();
    configMotors();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
        // Intake up
        // Stop All Motors
      }

      @Override
      public MMStateMachineState calcNextState() {
        // check intake flag ->
        if (runIntake) {
          return DropIntake;
        }
        return this;
      }
    };

    MMStateMachineState DropIntake = new MMStateMachineState("DropIntake") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // setPosition for intake motor(down)
        // setVelocity for rollers (in)
        // setVelocity for feeders(in)
        // run Command for shooters to spin up
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (!intakeBreakBeam.get()) {
          return Index;
        }
        return this;
      }
    };
    MMStateMachineState Index = new MMStateMachineState("Index") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // stop rollers
        // stop feeders
        // setPosition for intake motor(up)
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (runShoot) {
          return PrepareToShoot;
        }

        return this;
      }
    };

    MMStateMachineState PrepareToShoot = new MMStateMachineState("PrepareToShoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {

      }

      @Override
      public MMStateMachineState calcNextState() {
        // if at angle based on waypoint & at speed based on waypoint -> shoot

        return this;
      }
    };

    MMStateMachineState Shoot = new MMStateMachineState("Shoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // Run feeders based on waypoint
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
      public void transitionTo(MMStateMachineState previousState) {

      }

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
        //
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (timeInState >= shooterDelay) {
          return Idle;
        } else {
          return this;
        }
      }
    };
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
    MMConfigure.configureDevice(LeftMotor, genericConfig);
    MMConfigure.configureDevice(index1, genericConfig);
    MMConfigure.configureDevice(index2, genericConfig);
  }

  public void runMotors(double leftMotorSpeed, double rightMotorSpeed) {
    LeftMotor.setControl(leftVelVol.withVelocity(leftMotorSpeed));
    rightMotor.setControl(rightVelVol.withVelocity(rightMotorSpeed));
  }

  public void runLeftMotor(double leftMotorSpeed) {
    LeftMotor.setControl(leftVelVol.withVelocity(leftMotorSpeed));
  }

  public void runRightMotor(double rightMotorSpeed) {
    rightMotor.setControl(rightVelVol.withVelocity(rightMotorSpeed));
  }

  public void stopMotors() {
    LeftMotor.set(0);
    rightMotor.set(0);
  }

  public void stopLeftMotor() {
    LeftMotor.set(0);
  }

  public void stopRightMotor() {
    rightMotor.set(0);
  }

  public void runIndexers(double index1Speed, double index2Speed) {
    index1.setControl(leftVelVol.withVelocity(index1Speed));
    index2.setControl(leftVelVol.withVelocity(index2Speed));
  }

  public void stopIndexers() {
    index2.set(0);
    index1.set(0);
  }

  public void aimToSpeaker() {
    double distance = rc.navigation.getDistanceToSpeaker();

    MMWaypoint desiredWaypoint = firingSolution.calcSolution(distance);
    runLeftMotor(desiredWaypoint.getLeftVelocity());
    runRightMotor(desiredWaypoint.getRightVelocity());
    //
  }

  private void configIntakeRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.390869140625);
    MMConfigure.configureDevice(intakeRotateCanCoder, canConfig);
  }

  private void configShooterRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.390869140625);
    MMConfigure.configureDevice(shooterRotateCanCoder, canConfig);
  }

  private void configIntakeRotateMotor() {
    double cruiseVelocity = .5; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = (60.0 / 15.0) * 48.0;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

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
    double rotorToSensorRatio = (60.0 / 15.0) * 48.0;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

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
