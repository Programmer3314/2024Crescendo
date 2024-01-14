// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

// import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class MMController extends CommandXboxController {

    public double deadzone;

    public MMController(int port, double deadzone) {
        super(port);
        this.deadzone = deadzone;
    }

    // TODO: Add Scaling...
    // Add methods to set individual axis scales.
    // Use method chaining to make it easier to set multiple scales "at once"
    public double getLeftXSmoothed() {
        return smoothAxis(super.getLeftX());
    }

    public double getRightXSmoothed() {
        return smoothAxis(super.getRightX());
    }

    public double getLeftYSmoothed() {
        return smoothAxis(super.getLeftY());
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
}
