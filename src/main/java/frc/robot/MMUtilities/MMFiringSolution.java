// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

public class MMFiringSolution {
    double speakerHeight = 2.4;
    double pivotHeight = .127;
     

    private MMWaypoint[] waypoints;

    public MMFiringSolution(MMWaypoint... waypoints) {
        this.waypoints = waypoints;
    }

    // make Waypoint and MMFiringSolution use 
    // Angle, Left & Right velocities, and Index Speed. 
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

                // double shootAngle = Math.atan2((speakerHeight - pivotHeight), distance)/(2*Math.PI) + .26
                // ;
                // //34 degrees and .389 encoder value
                // //.593 radians .095+ .294 
                // //.455 encoder value and 58 degrees
                // //.16 radians
                // //.458 encoder value
                // //.385 encoder value
                // if(shootAngle > .458 ){
                //     shootAngle = .458;

                // }
                // if(shootAngle < .385){
                //     shootAngle = .385;
                // }

       // return new MMWaypoint(distance, shootAngle, desiredLeftVelocity, desiredRightVelocity, desiredVelocity);
        return new MMWaypoint(distance, desiredAngle, desiredLeftVelocity, desiredRightVelocity, desiredVelocity);
    }

}
