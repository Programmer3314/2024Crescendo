// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.MMUtilities;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public interface MMSwerveRequest {

    /**
     * Drives the swerve drivetrain in a field-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the rate at which their robot should rotate
     * about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class FieldCentricSlewRateLimitted implements SwerveRequest {
        /**
         * The velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         */
        public double RotationalRate = 0;
        /**
         * The allowable deadband of the request.
         */
        public double Deadband = 0;
        /**
         * The rotational deadband of the request.
         */
        public double RotationalDeadband = 0;

        /**
         * The number of modules (required to create slew rate filters)
         */
        public int ModuleCount = 0;

        /**
         * Filter speed
         */
        private boolean filterSpeed = false;
        public double speedPositiveSlewRate = 0;
        public double speedNegativeSlewRate = 0;
        private SlewRateLimiter[] speedSlewRateLimiters=null;

        /**
         * Filter angle
         */
        private boolean filterAngle = false;
        public Rotation2d angleSlewRate = new Rotation2d();
        private MMSwerveAngleSlewRateFilter[] angSlewRateLimiters;

        /**
         * The type of control request to use for the drive motor.
         */
        public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        /**
         * The type of control request to use for the steer motor.
         */
        public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

        /**
         * The last applied state in case we don't have anything to drive.
         */
        protected SwerveModuleState[] m_lastAppliedState = null;

        public FieldCentricSlewRateLimitted(int ModuleCount) {
            // throw new Exception("Incomplete Class FieldCentricSlewRateLimitted");
            // this.ModuleCount = ModuleCount;

        }

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double toApplyX = VelocityX;
            double toApplyY = VelocityY;
            double toApplyOmega = RotationalRate;
            if (Math.sqrt(toApplyX * toApplyX + toApplyY * toApplyY) < Deadband) {
                toApplyX = 0;
                toApplyY = 0;
            }
            if (Math.abs(toApplyOmega) < RotationalDeadband) {
                toApplyOmega = 0;
            }

            ChassisSpeeds speeds = ChassisSpeeds
                    .discretize(ChassisSpeeds.fromFieldRelativeSpeeds(toApplyX, toApplyY, toApplyOmega,
                            parameters.currentPose.getRotation()), parameters.updatePeriod);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            if (speedSlewRateLimiters == null) {
                this.speedSlewRateLimiters = new SlewRateLimiter[ModuleCount];
                this.angSlewRateLimiters = new MMSwerveAngleSlewRateFilter[ModuleCount];
                for (int i = 0; i < ModuleCount; i++) {
                    speedSlewRateLimiters[i] = new SlewRateLimiter(speedPositiveSlewRate, speedNegativeSlewRate, 0);
                    angSlewRateLimiters[i] = new MMSwerveAngleSlewRateFilter(this.angleSlewRate,modulesToApply[i].getCurrentState().angle);
                }
            }

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState sms = new SwerveModuleState(
                    speedSlewRateLimiters[i].calculate(states[i].speedMetersPerSecond),
                    angSlewRateLimiters[i].calculate(states[i].angle)
                    );
                modulesToApply[i].apply(sms, DriveRequestType, SteerRequestType);
            }

            return StatusCode.OK;
        }

        /**
         * Turns on the speed slew filter and sets the value.
         * 
         * @param slewRate
         * @return this request
         */
        public FieldCentricSlewRateLimitted withSpeedSlewRate(double slewRate) {
            this.filterSpeed = true;
            this.speedPositiveSlewRate = slewRate;
            this.speedNegativeSlewRate = -slewRate;
            return this;
        }

        /**
         * Turns on the speed slew filter and sets the value.
         * 
         * @param slewRate
         * @return this request
         */
        public FieldCentricSlewRateLimitted withSpeedSlewRate(double slewPositiveRate, double slewNegativeRate) {
            this.filterSpeed = true;
            this.speedPositiveSlewRate = slewPositiveRate;
            this.speedNegativeSlewRate = slewNegativeRate;
            return this;
        }

        /**
         * Turns on the angle slew filter and sets the value.
         * 
         * @param slewRate
         * @return this request
         */
        public FieldCentricSlewRateLimitted withAngleSlewRate(Rotation2d slewRate) {
            this.filterAngle = true;
            this.angleSlewRate = slewRate;
            return this;
        }

        /**
         * Sets the velocity in the X direction, in m/s.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         *
         * @param velocityX Velocity in the X direction, in m/s
         * @return this request
         */
        public FieldCentricSlewRateLimitted withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        /**
         * Sets the velocity in the Y direction, in m/s.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         *
         * @param velocityY Velocity in the Y direction, in m/s
         * @return this request
         */
        public FieldCentricSlewRateLimitted withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        /**
         * The angular rate to rotate at, in radians per second.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         *
         * @param rotationalRate Angular rate to rotate at, in radians per second
         * @return this request
         */
        public FieldCentricSlewRateLimitted withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        /**
         * Sets the allowable deadband of the request.
         *
         * @param deadband Allowable deadband of the request
         * @return this request
         */
        public FieldCentricSlewRateLimitted withDeadband(double deadband) {
            this.Deadband = deadband;
            return this;
        }

        /**
         * Sets the rotational deadband of the request.
         *
         * @param rotationalDeadband Rotational deadband of the request
         * @return this request
         */
        public FieldCentricSlewRateLimitted withRotationalDeadband(double rotationalDeadband) {
            this.RotationalDeadband = rotationalDeadband;
            return this;
        }

        /**
         * Sets the type of control request to use for the drive motor.
         *
         * @param driveRequestType The type of control request to use for the drive
         *                         motor
         * @return this request
         */
        public FieldCentricSlewRateLimitted withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
            this.DriveRequestType = driveRequestType;
            return this;
        }

        /**
         * Sets the type of control request to use for the steer motor.
         *
         * @param steerRequestType The type of control request to use for the steer
         *                         motor
         * @return this request
         */
        public FieldCentricSlewRateLimitted withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
            this.SteerRequestType = steerRequestType;
            return this;
        }
    }
}
