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
  public Shooter() {
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
        return Home;
      };
    };

    MMStateMachineState Home = new MMStateMachineState("Home") {

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
        // intake signal -> intake state
        // outake signal -> outake state
        // home signal -> home state
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
