// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.wpilibj.Timer;

public class MMStateMachine {
    double stateStartSeconds;
    double timeInState;

    MMStateMachineState currentState;
    boolean firstUpdate;

    public MMStateMachine setInitial(MMStateMachineState initialState) {
        currentState = initialState;
        firstUpdate = true;
        return this;
    }

    public MMStateMachine update() {
        double currentTime = Timer.getFPGATimestamp();
        timeInState = currentTime - stateStartSeconds;
        if (!firstUpdate) {
            MMStateMachineState nextState = currentState.calcNextState();
            if (currentState != nextState) {
                currentState.transistionFrom(nextState);
                stateStartSeconds = currentTime;
                timeInState = 0;
                nextState.transitionTo(currentState);
                currentState = nextState;
            }
        } else {
            firstUpdate = false;
        }
        currentState.doState();
        return this;
    }
}