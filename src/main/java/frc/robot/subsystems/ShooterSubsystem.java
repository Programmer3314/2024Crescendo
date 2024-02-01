// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;

public class ShooterSubsystem extends SubsystemBase {
  private TalonFX topMotor = new TalonFX(3, "CANIVORE");
  private TalonFX bottomMotor = new TalonFX(4, "CANIVORE");
  private TalonFX index1 = new TalonFX(1, "CANIVORE");
  private TalonFX index2 = new TalonFX(2, "CANIVORE");
  private static double topMotorSpeed = 0;
  private static double bottomMotorSpeed = 0;
  private static double currentLimit = 0;
  private static double index1Speed = 0;
  private static double index2Speed = 0;
  private VelocityVoltage topVelVol = new VelocityVoltage(0);
  private VelocityVoltage bottomVelVol = new VelocityVoltage(0);

  private boolean XP;
  private boolean XR;
  private boolean YP;
  private boolean YR;

  private ShooterStateMachine ssm = new ShooterStateMachine();

  /** Creates a new ExampleSubsystem. */
  public ShooterSubsystem() {
    SmartDashboard.putNumber("Top Motor Speed", topMotorSpeed);
    SmartDashboard.putNumber("Bottom Motor Speed", bottomMotorSpeed);
    SmartDashboard.putNumber("Index 1 Speed", index1Speed);
    SmartDashboard.putNumber("Index 2 Speed", index2Speed);

    SmartDashboard.putNumber("Current Limit", currentLimit);
    genericConfig();

    ssm.setInitial(ssm.Start);
  }

  public void setXP() {
    XP = true;
  }

  public void setXR() {
    XR = true;
  }

  public void setYP() {
    YP = true;
  }

  public void setYR() {
    YR = true;
  }

  public void genericConfig() {
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
    Robot.configureMotor(bottomMotor, genericConfig, currentLimit);
    Robot.configureMotor(topMotor, genericConfig, currentLimit);
    Robot.configureMotor(index1, genericConfig, currentLimit);
    Robot.configureMotor(index2, genericConfig, currentLimit);
  }

  public void setMotorSpeed() {
    index1Speed = SmartDashboard.getNumber("Index 1 Speed", topMotorSpeed);
    index2Speed = SmartDashboard.getNumber("Index 2 Speed", topMotorSpeed);
    topMotorSpeed = SmartDashboard.getNumber("Top Motor Speed", topMotorSpeed);
    bottomMotorSpeed = SmartDashboard.getNumber("Bottom Motor Speed", bottomMotorSpeed);
    currentLimit = SmartDashboard.getNumber("Current Limit", currentLimit);
    System.out.println("Just ran Set MOTOR SPEED");
    System.out.println("Top Motor Value " + topMotorSpeed);
    System.out.println("Bottom Motor Value " + bottomMotorSpeed);
    System.out.println("Index 1 Value " + index1Speed);
    System.out.println("Index 2 Value " + index2Speed);
    System.out.println("Current Limit " + currentLimit);
    genericConfig();
  }

  public void runMotors() {
    topMotor.setControl(topVelVol.withVelocity(topMotorSpeed));
    bottomMotor.setControl(bottomVelVol.withVelocity(bottomMotorSpeed));
  }

  public void runTopMotor() {
    topMotor.setControl(topVelVol.withVelocity(topMotorSpeed));
  }

  public void runBottomMotor() {
    bottomMotor.setControl(bottomVelVol.withVelocity(bottomMotorSpeed));
  }

  public void stopMotors() {
    topMotor.set(0);
    bottomMotor.set(0);
  }

  public void stopTopMotor() {
    topMotor.set(0);
  }

  public void stopBottomMotor() {
    bottomMotor.set(0);
  }

  public void runIndexers() {

    index1.setControl(topVelVol.withVelocity(index1Speed));
    index2.setControl(topVelVol.withVelocity(index2Speed));
  }

  public void stopIndexers() {
    index2.set(0);
    index1.set(0);
  }

  @Override
  public void periodic() {
     SmartDashboard.putNumber("Returned Top Motor Speed",
     topMotor.getVelocity().getValue());
     SmartDashboard.putNumber("Returned Bottom Motor Speed",
     bottomMotor.getVelocity().getValue());
    SmartDashboard.putNumber("Returned Index 1 Speed", index1.getVelocity().getValue());
    SmartDashboard.putNumber("Returned Index 2 Speed", index2.getVelocity().getValue());

    SmartDashboard.putNumber("Wanted Top Speed", topMotorSpeed);
    SmartDashboard.putNumber("Wanted Bottom Speed", bottomMotorSpeed);
    SmartDashboard.putNumber("Wanted Index 1 Speed", index1Speed);
    SmartDashboard.putNumber("Wanted Index 2 Speed", index2Speed);

    SmartDashboard.putBoolean("X Press", XP);
    SmartDashboard.putBoolean("X Release", XR);
    SmartDashboard.putBoolean("Y Press", YP);
    SmartDashboard.putBoolean("Y Release", YR);

    ssm.update();

    XP = false;
    XR = false;
    YP = false;
    YR = false;

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public class ShooterStateMachine extends MMStateMachine {

    MMStateMachineState Start = new MMStateMachineState("Start") {

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
      }

      @Override
      public MMStateMachineState calcNextState() {
        return NoRun;
      };
    };

    MMStateMachineState NoRun = new MMStateMachineState("NoRun") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {

        stopMotors();
      }

      @Override
      public MMStateMachineState calcNextState() {

        if (YP && XP) {
          return runBoth;
        }
        if (YP) {
          return runTop;
        }
        if (XP) {
          return runBtm;
        }

        return this;
      }
    };

    MMStateMachineState runTop = new MMStateMachineState("runTop") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runTopMotor();
        stopBottomMotor();
      }

      @Override
      public MMStateMachineState calcNextState() {

        if (YR && XP) {
          return runBtm;
        }
        if (YR) {
          return NoRun;
        }
        if (XP) {
          return runBoth;
        }

        return this;
      }
    };

    MMStateMachineState runBtm = new MMStateMachineState("runBtm") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runBottomMotor();
        stopTopMotor();
      }

      @Override
      public MMStateMachineState calcNextState() {

        if (YP && XR) {
          return runTop;
        }
        if (XR) {
          return NoRun;
        }
        if (YP) {
          return runBoth;
        }

        return this;
      }

    };
    MMStateMachineState runBoth = new MMStateMachineState("runBoth") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        runMotors();
      }

      @Override
      public MMStateMachineState calcNextState() {

        if (YR && XR) {
          return NoRun;
        }
        if (XR) {
          return runTop;
        }
        if (YR) {
          return runBtm;
        }

        return this;
      }
    };
  }
}
