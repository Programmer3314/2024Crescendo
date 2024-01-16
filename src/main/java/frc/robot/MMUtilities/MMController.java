// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class MMController extends CommandXboxController {
    // TODO: Implement deadzone chaining with individual deadbands(depending on the
    // joystick)

    public double deadzone;

    public double scaleXLeft = 1;
    public double scaleXRight = 1;
    public double scaleYLeft = 1;
    public double scaleYRight = 1;

    public MMController(int port, double deadzone) {
        super(port);
        this.deadzone = deadzone;
    }

    public MMController setScaleXLeft(double scale) {
        scaleXLeft = scale;
        return this;
    }

    public MMController setScaleXRight(double scale) {
        scaleXRight = scale;
        return this;
    }

    public MMController setScaleYLeft(double scale) {
        scaleYLeft = scale;
        return this;
    }

    public MMController setScaleYRight(double scale) {
        scaleYRight = scale;
        return this;
    }

    public double getLeftXSmoothed() {
        return smoothAxis(super.getLeftX()) * scaleXLeft;
    }

    public double getRightXSmoothed() {
        return smoothAxis(super.getRightX()) * scaleXRight;
    }

    public double getLeftYSmoothed() {
        return smoothAxis(super.getLeftY()) * scaleYLeft;
    }

    public double getRightYSmoothed() {
        return smoothAxis(super.getRightY()) * scaleYRight;
    }

    private double smoothAxis(double v) {
        if (v > -deadzone && v < deadzone) {
            v = 0;

        } else if (v >= deadzone) {
            v = (v - deadzone) / (1 - deadzone);
            v = v * v;

        } else if (v <= -deadzone) {
            v = ((v + deadzone) / (1 - deadzone));
            v = v * -v;
        }
        return v;
    }

    // TODO: Add triggers to change scale/deadzone
}
