// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

// TODO: The Shooter will have independant left/right velocity
// TODO: Change the calculation as well. 

/** Add your docs here. */
public class MMWaypoint {
    private double distance;
    private double angle;
    private double velocity;
    private double leftVelocity;
    private double rightVelocity;

    public MMWaypoint(double distance, double angle, double velocity) {
        this.distance = distance;
        this.angle = angle;
        this.velocity = velocity;
    }

    public MMWaypoint(double distance, double angle, double leftVelocity, double rightVelocity, double velocity) {
        this.distance = distance;
        this.angle = angle;
        this.leftVelocity = leftVelocity;
        this.rightVelocity = rightVelocity;
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

    public double getLeftVelocity() {
        return leftVelocity;
    }

    public double getRightVelocity() {
        return rightVelocity;
    }
}
