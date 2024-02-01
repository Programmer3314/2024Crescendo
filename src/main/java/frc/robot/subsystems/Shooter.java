// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMStateMachine;
import frc.robot.MMUtilities.MMStateMachineState;

public class Shooter extends SubsystemBase {
  RobotContainer rc;
  
  /** Creates a new Shooter. */
  public Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public class ShooterStateMachine extends MMStateMachine {

    MMStateMachineState Start = new MMStateMachineState("Start") {

      @Override
      public void transitionFrom(MMStateMachineState nextState) {
      }

      @Override
      public MMStateMachineState calcNextState() {
        return Home;
      };
    };

    MMStateMachineState Home = new MMStateMachineState("Home"){

      @Override 
      public void transitionTo(MMStateMachineState previousState) {

        // Intake up 
        // Shooter down
      }
      @Override
      public MMStateMachineState calcNextState() {
        return NoRun;
      }
    };
 
    MMStateMachineState NoRun = new MMStateMachineState("NoRun") {

      @Override
      public void transitionTo(MMStateMachineState previousState) {
        // stop motors
        // stop intake
        // 
        // stopMotors();
      }

      @Override
      public MMStateMachineState calcNextState() {
        //  intake signal -> intake state 
        //  outake signal -> outake state
        //  home signal -> home state
        // 
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
  }
}
