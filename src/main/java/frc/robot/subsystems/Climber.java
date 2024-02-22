// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  RobotContainer rc;
  CANcoder leftCanCoder;
  CANcoder rightCanCoder;
  Pigeon2 pigeon;
  TalonFX climbMotor;
  // Should the pigeon be owned in the navigation or here
  // it's only used in climber(for now), so this makes sense(for now)

  public Climber(RobotContainer rc) {
    this.rc = rc;

    pigeon = new Pigeon2(1, "CANIVORE");

    leftCanCoder = new CANcoder(1, "CANIVORE");
    rightCanCoder = new CANcoder(2, "CANIVORE");

    climbMotor = new TalonFX(18, "CANIVORE");
  }

  @Override
  public void periodic() {

    // configCanCoders()

  }

  public void configCanCoders() {
    CANcoderConfiguration rightCanConfig = new CANcoderConfiguration();
    rightCanConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)// one of them s
        .withMagnetOffset(0);

    CANcoderConfiguration leftCanConfig = new CANcoderConfiguration();
    leftCanConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(0);
    MMConfigure.configureDevice(leftCanCoder, leftCanConfig);
    MMConfigure.configureDevice(rightCanCoder, rightCanConfig);
  }

  public void configClimbMotor() {

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
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();
    climbConfig.CurrentLimits.SupplyCurrentLimit = 4;
    climbConfig.MotorOutput
        .withNeutralMode(NeutralModeValue.Coast);
    climbConfig.MotionMagic
        .withMotionMagicCruiseVelocity(cruiseVelocity)
        .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
        .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
    climbConfig.Slot0
        .withKS(0) // voltage to overcome static friction
        .withKV(feedForwardVoltage)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withKP(16)
        .withKI(0)
        .withKD(0);
    // climbConfig.Feedback-- maybe this is a good opportunity to use a synced
    // encoder as
    // opposed to the fused.
    // .withFeedbackRemoteSensorID(intakeRotateCanCoder.getDeviceID())
    // .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
    // .withSensorToMechanismRatio(1)
    // .withRotorToSensorRatio(rotorToSensorRatio);

    MMConfigure.configureDevice(climbMotor, climbConfig);
  }

  private class ClimbStateMachine extends MMStateMachine {

    MMStateMachineState Start = new MMStateMachineState("Start") {

      @Override
      public MMStateMachineState calcNextState() {
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };
    MMStateMachineState Idle = new MMStateMachineState("Idle") {

      @Override
      public MMStateMachineState calcNextState() {
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };
    MMStateMachineState Align = new MMStateMachineState("Align") {

      @Override
      public MMStateMachineState calcNextState() {
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };
    MMStateMachineState Climb = new MMStateMachineState("Climb") {

      @Override
      public MMStateMachineState calcNextState() {
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };
  }
}
