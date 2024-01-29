// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class MMController extends CommandXboxController {

    public double deadzoneLeftY;
    public double deadzoneLeftX;
    public double deadzoneRightY;
    public double deadzoneRightX;

    public double scaleXLeft = 1;
    public double scaleXRight = 1;
    public double scaleYLeft = 1;
    public double scaleYRight = 1;

    public MMController(int port) {
        super(port);
    }

    public MMController setDeadzone(double deadzone) {
        this.deadzoneLeftY = deadzone;
        this.deadzoneLeftX = deadzone;
        this.deadzoneRightX = deadzone;
        this.deadzoneRightY = deadzone;
        return this;
    }

    public MMController setDeadzoneLeftY(double deadzoneLeftY) {
        this.deadzoneLeftY = deadzoneLeftY;
        return this;
    }

    public MMController setDeadzoneLeftX(double deadzoneLeftX) {
        this.deadzoneLeftX = deadzoneLeftX;
        return this;
    }

    public MMController setDeadzoneRightY(double deadzoneRightY) {
        this.deadzoneRightY = deadzoneRightY;
        return this;
    }

    public MMController setDeadzoneRightX(double deadzoneRightX) {
        this.deadzoneRightX = deadzoneRightX;
        return this;
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
        return smoothAxis(super.getLeftX(), deadzoneLeftX) * scaleXLeft;
    }

    public double getRightXSmoothed() {
        return smoothAxis(super.getRightX(), deadzoneRightX) * scaleXRight;
    }

    public double getLeftYSmoothed() {
        return smoothAxis(super.getLeftY(), deadzoneLeftY) * scaleYLeft;
    }

    public double getRightYSmoothed() {
        return smoothAxis(super.getRightY(), deadzoneRightY) * scaleYRight;
    }

    private double smoothAxis(double v, double deadzone) {
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

}
