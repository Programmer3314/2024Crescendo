// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class MMTurnPIDController extends MMPIDController {

    public MMTurnPIDController() {
        super(6.5, 0, .05, Math.PI / 2.0, Math.toRadians(1), true);
    }

    public MMTurnPIDController(boolean isFastTurn) {
        super(isFastTurn ? 12 : 6.5, 0, 1.15, Math.PI, Math.toRadians(1), true);
    }

    public double execute(Rotation2d rotation) {
        return execute(rotation.getRadians());
    }

    public void initialize(Rotation2d rotation) {
        initialize(rotation.getRadians());
    }
}
