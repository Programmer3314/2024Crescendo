// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

public class MMStateMachineState {
    String name;

    public MMStateMachineState(String name) {
        this.name = name;
    }

    public MMStateMachineState calcNextState() {
        return this;
    };

    public void transitionTo(MMStateMachineState previousState) {
    };

    public void transistionFrom(MMStateMachineState nextState) {
    };

    public void doState() {
    };
}