// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.enums.SignalSelection;

/** Add your docs here. */
public class MMSignalLight extends SubsystemBase {
    private Solenoid coneLight;
    private Solenoid cubeLight;
    private PneumaticsControlModule pneumaticsControlModule;
    private int currentCycle;
    private int blinkDuration;

    public MMSignalLight() {
        pneumaticsControlModule = new PneumaticsControlModule(1);
        // coneLight = pneumaticsControlModule.makeDoubleSolenoid(
        // 6, 7);
        coneLight = pneumaticsControlModule.makeSolenoid(6);
        cubeLight = pneumaticsControlModule.makeSolenoid(7);
    }

    @Override
    public void periodic() {
        setLight(RobotContainer.signalSelection);
    }

    public void setLight(SignalSelection selection) {
        currentCycle++;
        switch (selection) {
            case Cone_Blink_Slow:
                blinkDuration = 20;
                set_cone_blink_slow();
                break;
            case Cube_Blink_Slow:
                blinkDuration = 20;
                set_cube_blink_slow();
                break;
            case All_Blink_Slow:

                break;
            case Cone_Blink_Fast:
                blinkDuration=10;
                break;
            case Cube_Blink_Fast:
                blinkDuration=10;

                break;
            case All_Blink_Fast:

                break;
            case Cube_Solid:
                set_cube_on();
                break;
            case Cone_Solid:
                set_cone_on();
                break;
            case Cone_Off:
                set_cone_off();
                break;
            case Cube_Off:
                set_cube_off();
                break;
            default:
                set_all_off();
                break;
        }

    }

    /*
     * public void setLightCube() {
     * shineLight.set(Value.kReverse);
     * }
     * 
     * public void setLightCone() {
     * shineLight.set(Value.kForward);
     * }
     */
    private void set_all_off() {
        cubeLight.set(false);
        coneLight.set(false);
    }

    private void set_cube_off() {
        cubeLight.set(false);
    }

    private void set_cone_off() {
        coneLight.set(false);
    }

    private void set_cone_on() {
        coneLight.set(true);
    }

    private void set_cube_on() {
        cubeLight.set(true);
    }

    private void set_cube_blink_slow() {
        if (currentCycle >= blinkDuration) {
            set_cube_on();
        } else {
            set_cube_off();
        }
        blinkDuration %= 40;
    }

    private void set_cone_blink_slow() {
        if (currentCycle >= blinkDuration) {
            set_cone_on();
        } else {
            set_cone_off();
        }
        currentCycle %= 40;
    }

    private void set_cone_blink_fast() {
        if (currentCycle >= blinkDuration) {
            set_cube_on();
        } else {
            set_cube_off();
        }
        blinkDuration %= 20;
    }
}
