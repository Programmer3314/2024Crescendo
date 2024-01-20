// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class MMTurnPIDController extends MMPIDController {

    public MMTurnPIDController() {
        super(5, 0, 0, Math.PI / 2.0, Math.toRadians(3), true);
        // 2.5
    }

    public MMTurnPIDController(boolean isFastTurn) {
        super(isFastTurn ? 8 : 5, 0, 0, Math.PI / 2.0, Math.toRadians(3), true);
        // 2.5
    }

    public double execute(Rotation2d rotation) {
        return execute(rotation.getRadians());
    }

    public void initialize(Rotation2d rotation) {
        initialize(rotation.getRadians());
    }
}
