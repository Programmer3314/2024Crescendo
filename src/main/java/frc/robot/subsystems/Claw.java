// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMRollingAvg;
import frc.robot.MMUtilities.MMWaypoint;

public class Claw extends SubsystemBase {

    MMFiringSolution firingSolution = new MMFiringSolution(
            new MMWaypoint(Units.feetToMeters(3), -35, 50),
            new MMWaypoint(Units.inchesToMeters(93), -44, 70),
            new MMWaypoint(Units.inchesToMeters(110), -46, 70));
    TalonFX clawMotor = new TalonFX(14, "CANIVORE");
    DigitalInput brakeSensor = new DigitalInput(6);
    VoltageOut voltageRequest = new VoltageOut(0);

    TalonFX armRotateMotor = new TalonFX(12, "CANIVORE"); 
    CANcoder armRotateCanCoder = new CANcoder(5, "CANIVORE");

    TalonFX armExtendMotor = new TalonFX(11, "CANIVORE");
    private final MotionMagicVoltage armExtendMotionMagicVoltage = new MotionMagicVoltage(0, true, 0, 0, false, false,
            false);
    private final MotionMagicVoltage armRotateMotionMagicVoltage = new MotionMagicVoltage(0);
            
    // private final MotionMagicTorqueCurrentFOC armExTorqueCurrentFOC = new
    // MotionMagicTorqueCurrentFOC(0,
    // 0, 0, false, false , false);
    // private final VelocityVoltage armExtendVelocity = new VelocityVoltage(0);

    DoubleSolenoid pinch = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM,
            5, 4);

    MMRollingAvg avgDis = new MMRollingAvg(10);

    public RobotContainer rc;

    public Claw(RobotContainer rc) {
        configClawMotor();
        configArmExtendMotor();
        configArmRotateCanCoder();
        configArmRotateMotor();
        armExtendMotor.setPosition(0);
        armExtensionRot(0);
        this.rc = rc;
    }

    private void configArmExtendMotor() {
        double cruiseVelocity = 35; // revolutions/second 
        double timeToReachCruiseVelocity = .35; // seconds
        double timeToReachMaxAcceleration = .1; // seconds
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        cfg.MotionMagic
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
                .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
        cfg.Slot0
                .withKS(0.25) // voltage to overcome static friction
                .withKV(0.12) // should be 12volts/(max speed in rev/sec) Typical Falcon 6000revs/min or 100
                              // revs/sec
                .withKA(0.01) // "arbitrary" amount to provide crisp response
                .withKG(0) // gravity can be used for elevator or arm
                .withKP(6) // 2 revs yields 12 volts
                .withKI(0)
                .withKD(0.1);
        MMConfigure.configureDevice(armExtendMotor, cfg);
    }

    private void configArmRotateCanCoder() {
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        canConfig.MagnetSensor
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
                .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                .withMagnetOffset(-0.390869140625);
        MMConfigure.configureDevice(armRotateCanCoder, canConfig);
    }

    private void configArmRotateMotor() {
        double cruiseVelocity = .5; // Sensor revolutions/second 
        double timeToReachCruiseVelocity = .4; // seconds
        double timeToReachMaxAcceleration = .2; // seconds
        // TODO: review feed forward formulas
        double maxSupplyVoltage = 12; // Max supply 
        double staticFrictionVoltage = 1; // 
        double rotorToSensorRatio = (60.0 / 15.0) * 48.0;
        double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec 
        double maxSensorVelocity = maxRotorVelocity/rotorToSensorRatio; // Max speed in sensor units/sec
        double feedForwardVoltage = (maxSupplyVoltage-staticFrictionVoltage)/maxSensorVelocity; // Full Voltage/Max Sensor Velocity

        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        cfg.MotionMagic
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
                .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
        cfg.Slot0
                .withKS(1) // voltage to overcome static friction
                .withKV(feedForwardVoltage) 
                .withKA(0) // "arbitrary" amount to provide crisp response
                // TODO: Let's play with kG
                .withKG(0) // gravity can be used for elevator or arm
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(12) 
                .withKI(0)
                .withKD(2);
        cfg.Feedback
                .withFeedbackRemoteSensorID(armRotateCanCoder.getDeviceID()) 
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                .withSensorToMechanismRatio(1)
                .withRotorToSensorRatio(rotorToSensorRatio);
        MMConfigure.configureDevice(armRotateMotor, cfg);
    }

    private void configClawMotor() {
        TalonFXConfiguration clawCfg = new TalonFXConfiguration();
        clawCfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        MMConfigure.configureDevice(clawMotor, clawCfg);
    }

    @Override
    public void periodic() {
        getDesiredWaypoint();
        //distExtender();
        SmartDashboard.putNumber("Arm Rotations", armExtendMotor.getPosition().getValue());
        SmartDashboard.putNumber("Arm Velocity", armExtendMotor.getVelocity().getValue());
    }

    private void getDesiredWaypoint() {
        Translation2d target = MMField.getBlueTranslation(new Translation2d((1.36 - Units.feetToMeters(3)), 5.55));
        double distance = rc.drivetrain.getState().Pose.getTranslation().minus(target).getNorm();
        MMWaypoint desiredWaypoint = firingSolution.calcSolution(distance);

        SmartDashboard.putNumber("Velocity", desiredWaypoint.getVelocity());
        SmartDashboard.putNumber("Angle", desiredWaypoint.getAngle());
        SmartDashboard.putNumber("Distance To Target", distance);
    }

    public void distExtender() {
        Pose2d currentPose = rc.drivetrain.getState().Pose;
        Translation2d target = MMField.getBlueTranslation(new Translation2d(1.36, 5.55));
        double distance = currentPose.getTranslation().minus(target).getNorm();
        distance *= (20 / 2);
        distance = MathUtil.clamp(distance, 0, 20);
        distance = avgDis.update(distance);
        armExtensionIn(distance);
        SmartDashboard.putNumber("Arm Extend Distance", distance);
    }

    public void armExtensionRot(double rotations) {
        armExtendMotor.setControl(armExtendMotionMagicVoltage.withPosition(-rotations));
    }

    public void armRotationRot(double rotations) {
        armRotateMotor.setControl(armRotateMotionMagicVoltage.withPosition(rotations));
    }

    public void armExtensionIn(double inches) {
        double rotations = inches * (42 / 38);
        armExtensionRot(rotations);
    }

    public void openClaw() {
        pinch.set(Value.kReverse);
    }

    public void closeClaw() {
        pinch.set(Value.kForward);
    }

    public void Out() {
        clawMotor.setControl(voltageRequest.withOutput(12.0));
    }

    public void In() {
        clawMotor.setControl(voltageRequest.withOutput(-6.0));
    }

    public void Stop() {
        clawMotor.setControl(voltageRequest.withOutput(0.0));
    }

    public boolean isBroken() {
        return !brakeSensor.get();
    }
}
