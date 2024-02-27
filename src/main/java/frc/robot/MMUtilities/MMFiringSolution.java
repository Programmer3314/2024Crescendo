// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;

public class MMFiringSolution {
    RobotContainer rc;
    double speakerHeight = Units.inchesToMeters(83);
    double pivotHeight = Units.inchesToMeters(5);

    private MMWaypoint[] waypoints;

    public MMFiringSolution(RobotContainer rc, MMWaypoint... waypoints) {
        this.rc = rc;
        this.waypoints = waypoints;
    }

    // make Waypoint and MMFiringSolution use
    // Angle, Left & Right velocities, and Index Speed.

    public MMWaypoint calcSolution(double distance) {

        MMWaypoint res = null;
        MMWaypoint bottomRef = null;
        MMWaypoint topRef = null;

        if (distance <= waypoints[0].getDistance()) {
            res = waypoints[0];
        } else if (distance >= waypoints[waypoints.length - 1].getDistance()) {
            res = waypoints[waypoints.length - 1];
        } else {
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

            res = new MMWaypoint(distance, desiredAngle, desiredLeftVelocity, desiredRightVelocity, desiredVelocity);
        }

        // Alternate angle calc...
        // 34 degrees and .389 encoder value
        // .593 radians .095+ .294
        // .455 encoder value and 58 degrees
        // .16 radians
        // .458 encoder value
        // .385 encoder value
        double shootAngle = Math.atan2((speakerHeight - pivotHeight), distance) / (2 * Math.PI) + .306;// +.31
        SmartDashboard.putNumber("fsShootAnglePrior", shootAngle);
        if (shootAngle > .458) {
            shootAngle = .458;
        }
        if (shootAngle < .38) {
            shootAngle = .38;
        }
        SmartDashboard.putNumber("fsRise", (speakerHeight - pivotHeight));
        SmartDashboard.putNumber("fsRun", distance);
        SmartDashboard.putNumber("fsAtan2", Math.atan2((speakerHeight - pivotHeight), distance));
        SmartDashboard.putNumber("fsDistance", distance);
        SmartDashboard.putNumber("fsShootAngle", shootAngle);
        SmartDashboard.putNumber("fsDesiredAngle", res.getAngle());
        SmartDashboard.putNumber("fsEncoderValue", rc.shooterSubsystem.getShooterAngle());

        return new MMWaypoint(distance, shootAngle, res.getLeftVelocity(), res.getRightVelocity(),
                res.getVelocity());
        // return new MMWaypoint(distance, desiredAngle, desiredLeftVelocity,
        // desiredRightVelocity, desiredVelocity);
    }
}
