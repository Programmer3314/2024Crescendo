// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMRollingAvg;


// TODO: get rid of Claw


public class Claw extends SubsystemBase {

    TalonFX clawMotor = new TalonFX(14, "CANIVORE");
    DigitalInput brakeSensor = new DigitalInput(6);
    VoltageOut voltageRequest = new VoltageOut(0);

    TalonFX armExtendMotor = new TalonFX(11, "CANIVORE");
    private final MotionMagicVoltage armExtendMotionMagicVoltage = new MotionMagicVoltage(0, true, 0, 0, false, false,
            false);

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
        armExtendMotor.setPosition(0);
        armExtensionRot(0);
        this.rc = rc;
    }

    private void configArmExtendMotor() {
        double cruiseVelocity = 35; // revolutions/second
        double timeToReachCruiseVelocity = .35; // seconds
        double timeToReachMaxAcceleration = .1; // seconds
        double maxSupplyVoltage = 12; // Max supply
        double staticFrictionVoltage = 1; //
        double rotorToSensorRatio = 1;
        double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
        double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
        double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        cfg.MotionMagic
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(cruiseVelocity / timeToReachCruiseVelocity)
                .withMotionMagicJerk(cruiseVelocity / timeToReachCruiseVelocity / timeToReachMaxAcceleration);
        cfg.Slot0
                .withKS(0.25) // voltage to overcome static friction
                .withKV(feedForwardVoltage) // should be 12volts/(max speed in rev/sec) Typical Falcon 6000revs/min or
                                            // 100
                // revs/sec
                .withKA(0.01) // "arbitrary" amount to provide crisp response
                .withKG(0) // gravity can be used for elevator or arm
                .withKP(6) // 2 revs yields 12 volts
                .withKI(0)
                .withKD(0.1);
        MMConfigure.configureDevice(armExtendMotor, cfg);
    }

    private void configClawMotor() {
        TalonFXConfiguration clawCfg = new TalonFXConfiguration();
        clawCfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        MMConfigure.configureDevice(clawMotor, clawCfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rotations", armExtendMotor.getPosition().getValue());
        SmartDashboard.putNumber("Arm Velocity", armExtendMotor.getVelocity().getValue());
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
