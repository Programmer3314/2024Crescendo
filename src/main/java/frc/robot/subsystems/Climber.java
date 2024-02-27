// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;
import frc.robot.subsystems.Shooter.ShooterStateMachine;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  RobotContainer rc;
  CANcoder leftCanCoder;
  CANcoder rightCanCoder;
  Pigeon2 pigeon;
  TalonFX climbMotor;
  private ClimbStateMachine csm = new ClimbStateMachine();
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();

  private final MotionMagicVoltage climbMotionMagicVoltage = new MotionMagicVoltage(0);
  double climbDownPosition = 0.047;
  double climbUpPosition = .28;
  double climbEngaged = .27;
  double climbSlowPos = .15;
  double elevatorSafety = .20;
  double climbPositionMargin = 0;

  int idleCounter = 0;

  boolean runClimb;
  boolean runTrap;
  boolean unwindClimb;

  MotionMagicVelocityVoltage climbMagicVelocityVoltage = new MotionMagicVelocityVoltage(0);
  MotionMagicVoltage climbMagicVol = new MotionMagicVoltage(0);

  // Should the pigeon be owned in the navigation or here
  // it's only used in climber(for now), so this makes sense(for now)

  public Climber(RobotContainer rc) {
    this.rc = rc;
    csm = new ClimbStateMachine();
    csm.setInitial(csm.Start);
    // TODO: Access the pidgeon via rc.drivetrain.getpidgeon()
    pigeon = new Pigeon2(1, "CANIVORE");

    leftCanCoder = new CANcoder(7, "CANIVORE");
    rightCanCoder = new CANcoder(8, "CANIVORE");

    climbMotor = new TalonFX(18, "CANIVORE");

    // TODO: configs...
    configClimbMotor();
    configCanCoders();
  }

  @Override
  public void periodic() {
    csm.update();
    SmartDashboard.putString("ClimbState", csm.currentState.getName());
    SmartDashboard.putNumber("Idle Counter", idleCounter);
    // configCanCoders()

  }

  private class ClimbStateMachine extends MMStateMachine {

    MMStateMachineState Start = new MMStateMachineState("Start") {

      @Override
      public MMStateMachineState calcNextState() {
        return Idle;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
        stopClimb();
      }
    };

    MMStateMachineState Idle = new MMStateMachineState("Idle") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        setClimbFlag(false);
        setTrapFlag(false);
        setClimbUnwindFlag(false);
        idleCounter++;
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (unwindClimb) {
          return UnwindClimb;
        }
        if (runClimb) {
          return ClawsUp;
        }
        if (runTrap) {
          return ClawsUp;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };

    MMStateMachineState ClawsUp = new MMStateMachineState("ClawsUp") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runClimbSlow();
        //
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() >= climbUpPosition
            && rightCanCoder.getAbsolutePosition().getValue() >= climbUpPosition) {
          return Engage;
        }
        return this;

      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
        setClimbPos();
      }
    };

    MMStateMachineState Engage = new MMStateMachineState("Engage") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rc.drivetrain.setControl(drive.withVelocityX(-.25).withVelocityY(0).withRotationalRate(0));
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() <= climbEngaged
            || rightCanCoder.getAbsolutePosition().getValue() <= climbEngaged) {
          return Climb;
        }
        return this;

      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
        rc.drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      }
    };

    MMStateMachineState Climb = new MMStateMachineState("Climb") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runClimbSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() <= elevatorSafety
            || rightCanCoder.getAbsolutePosition().getValue() <= elevatorSafety) {
          return RaiseElevator;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };

    MMStateMachineState RaiseElevator = new MMStateMachineState("Raise Elevator ") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rc.shooterSubsystem.setElevatorUpTrap();
        runClimbFast();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() <= climbSlowPos
            || rightCanCoder.getAbsolutePosition().getValue() <= climbSlowPos) {
          return SlowDown;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };

    MMStateMachineState SlowDown = new MMStateMachineState("SlowDown") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rc.shooterSubsystem.setElevatorUpTrap();
        runClimbSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() <= climbDownPosition
            || rightCanCoder.getAbsolutePosition().getValue() <= climbDownPosition) {
          return CheckElevator;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
        setClimbPos();
      }
    };
    MMStateMachineState CheckElevator = new MMStateMachineState("Check Elevator") {
      @Override
      public void transitionTo(MMStateMachineState previousState) {
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (rc.shooterSubsystem.isInMargin(rc.shooterSubsystem.getElevatorPosition(),
            rc.shooterSubsystem.elevatorTrapPosition, rc.shooterSubsystem.elevatorPositionMargin)) {
          return ElevatorPassNoteAbove;
        }
        return this;
      };

      @Override
      public void transitionFrom(MMStateMachineState NextState) {
      }
    };

    MMStateMachineState ElevatorPassNoteAbove = new MMStateMachineState("ElevatorPassNoteAbove") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rc.shooterSubsystem.runElevatorBeltUpSlow();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (rc.shooterSubsystem.elevatorBreakBeam.get()) {
          return ElevatorShoot;
        }
        return this;
      }
    };

    MMStateMachineState ElevatorShoot = new MMStateMachineState("ElevatorShoot") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        rc.shooterSubsystem.runElevatorBeltShoot();
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
        rc.shooterSubsystem.stopElevatorBelt();
      }
    };

    MMStateMachineState UnwindClimb = new MMStateMachineState("UnwindClimb") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runClimbNeg();
      }

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() >= .24) {
          return ResetClimb;
        }
        return this;
      }
    };

    MMStateMachineState ResetClimb = new MMStateMachineState("ResetClimb") {

      @Override
      public MMStateMachineState calcNextState() {
        if (leftCanCoder.getAbsolutePosition().getValue() <= .07
            || rightCanCoder.getAbsolutePosition().getValue() <= .07) {
          return Idle;
        }
        return this;
      }

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
        stopClimb();
        setClimbUnwindFlag(false);
      }
    };

  }

  public void runClimbNeg() {
    climbMotor.setControl(climbMagicVelocityVoltage.withVelocity(-10));
  }

  public void runClimbSlow() {
    climbMotor.setControl(climbMagicVelocityVoltage.withVelocity(20));
  }

  public void runClimbFast() {
    climbMotor.setControl(climbMagicVelocityVoltage.withVelocity(40));
  }

  public void stopClimb() {
    climbMotor.setControl(climbMagicVelocityVoltage.withVelocity(0));
  }

  public Climber setClimbFlag(boolean run) {
    runClimb = run;
    return this;
  }

  public void setClimbPos() {
    climbMotor.setControl(climbMagicVol.withPosition(climbMotor.getPosition().getValue()));
  }

  public Climber setTrapFlag(boolean run) {
    runTrap = run;
    return this;
  }

  public int getIdleCounter() {
    return idleCounter;
  }

  public void configCanCoders() {
    CANcoderConfiguration rightCanConfig = new CANcoderConfiguration();
    rightCanConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)// one of them s
        .withMagnetOffset(-0.7);

    CANcoderConfiguration leftCanConfig = new CANcoderConfiguration();
    leftCanConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        .withMagnetOffset(0.023);
    MMConfigure.configureDevice(leftCanCoder, leftCanConfig);
    MMConfigure.configureDevice(rightCanCoder, rightCanConfig);
  }

  public void configClimbMotor() {

    double cruiseVelocity = 10; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .125; // seconds
    double timeToReachMaxAcceleration = .05; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //

    // double sensorLow = 0.04;
    // double sensorHigh = 0.84;
    // double rotorLow = -9.8;
    // double rotorHigh = -0.6;
    double calcRotorToSensor = 1; // (rotorHigh - rotorLow) / (sensorHigh - sensorLow);

    double rotorToSensorRatio = calcRotorToSensor;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity
    TalonFXConfiguration climbConfig = new TalonFXConfiguration();

    // climbConfig.CurrentLimits.SupplyCurrentLimit = 40;
    climbConfig.MotorOutput
        .withNeutralMode(NeutralModeValue.Brake);
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
        .withKP(.3)
        .withKI(0)
        .withKD(0);

    MMConfigure.configureDevice(climbMotor, climbConfig);
  }

  public void resetStateMachine() {
    csm.setInitial(csm.Start);
  }

  public boolean isInMargin(double value1, double value2, double margin) {
    return Math.abs(value1 - value2) <= margin;
  }

  public void setClimbUnwindFlag(boolean flag) {
    unwindClimb = flag;
  }
}
