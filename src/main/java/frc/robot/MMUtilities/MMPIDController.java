// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class MMPIDController {
    private PIDController pidController;
    private double maxCorrection;

    public MMPIDController(double kP, double kI, double kD, double maxCorrection, double targetMargin,
            boolean isCircular) {
        this.maxCorrection = maxCorrection;

        pidController = new PIDController(kP, kI, kD);
        if (isCircular) {
            pidController.enableContinuousInput(-Math.PI, Math.PI);
        }
        pidController.setTolerance(targetMargin);
    }

    public void initialize(double setPoint) {
        pidController.setSetpoint(setPoint);
    }

    public double execute(double currentMeasurement) {
        double correction = pidController.calculate(currentMeasurement);
        if (correction > maxCorrection) {
            correction = maxCorrection;
        }
        if (correction < -maxCorrection) {
            correction = -maxCorrection;
        }
        return correction;
    }

    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
