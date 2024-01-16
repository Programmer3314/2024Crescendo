// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

/** Add your docs here. */
public class MMWaypoint {
    private double distance;
    private double angle;
    private double velocity;

    public MMWaypoint(double distance, double angle, double velocity) {
        this.distance = distance;
        this.angle = angle;
        this.velocity = velocity;
    }

    public double getDistance() {
        return distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getVelocity() {
        return velocity;
    }
}
