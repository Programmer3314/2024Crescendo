// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

/** Add your docs here. */
public class MMFiringSolution {

    private MMWaypoint[] waypoints;

    public MMFiringSolution(MMWaypoint... waypoints) {
        this.waypoints = waypoints;
    }

    public MMWaypoint calcSolution(double distance) {
        MMWaypoint bottomRef = null;
        MMWaypoint topRef = null;
        if (distance <= waypoints[0].getDistance()) {
            return waypoints[0];
        }
        if (distance >= waypoints[waypoints.length - 1].getDistance()) {
            return waypoints[waypoints.length - 1];
        }

        for (int i = 1; i < waypoints.length; i++) {// find the reference waypoints
            if (distance < waypoints[i].getDistance()) {
                topRef = waypoints[i];
                bottomRef = waypoints[i - 1];
                break;
            }
        }
        double refDistance = topRef.getDistance() - bottomRef.getDistance();
        double scale = (distance - bottomRef.getDistance()) / refDistance;
        double desiredAngle = (topRef.getAngle() - bottomRef.getAngle()) * scale + bottomRef.getAngle();
        double desiredVelocity = (topRef.getVelocity() - bottomRef.getVelocity()) * scale + bottomRef.getVelocity();

        double desiredLeftVelocity = (topRef.getLeftVelocity() - bottomRef.getLeftVelocity()) * scale
                + bottomRef.getLeftVelocity();
        double desiredRightVelocity = (topRef.getRightVelocity() - bottomRef.getRightVelocity()) * scale
                + bottomRef.getRightVelocity();

        return new MMWaypoint(distance, desiredAngle, desiredLeftVelocity, desiredRightVelocity, desiredVelocity);
    }

}
