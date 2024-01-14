// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
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
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Claw extends SubsystemBase {

    TalonFX clawMotor = new TalonFX(14, "CANIVORE");
    DigitalInput brakeSensor = new DigitalInput(6);
    VoltageOut voltageRequest = new VoltageOut(0);

    TalonFX armExtendMotor = new TalonFX(11, "CANIVORE");
    private final MotionMagicVoltage armExtendMotionMagicVoltage = new MotionMagicVoltage(0, true, 0, 0, false, false, false);
    private final MotionMagicTorqueCurrentFOC armExTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0,
            0, 0, false, false , false);
    //private final VelocityVoltage armExtendVelocity = new VelocityVoltage(0);

    DoubleSolenoid pinch = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM,
            5, 4);
public RobotContainer rc; 

    public Claw(RobotContainer rc) {
        configClawMotor();
        configArmExtendMotor();
        armExtendMotor.setPosition(0);
        armExtensionRot(0);
        this.rc = rc;
    }

    // TODO: WARNING Constants changed = review formulas for constants
    // TODO: Review configs moved out of constructor with Refactor,Extract Method
    // TODO: Review use of .with... formatting for readability
    // TODO: Must graph Velocity/Position to check tuning
    // TODO: Check extension in Inches
    private void configArmExtendMotor() {
        double cruiseVelocity = 35; // revolutions/second
        double timeToReachCruiseVelocity = .35; // seconds
        double timeToReachMaxAcceleration = .1; // seconds
        TalonFXConfiguration cfg = new TalonFXConfiguration();
        cfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        cfg.MotionMagic
                .withMotionMagicCruiseVelocity(cruiseVelocity)
                .withMotionMagicAcceleration(cruiseVelocity/timeToReachCruiseVelocity)
                .withMotionMagicJerk(cruiseVelocity/timeToReachCruiseVelocity/timeToReachMaxAcceleration);
        cfg.Slot0
                .withKS(0.25)  // voltage to overcome static friction
                .withKV(0.12)  // should be 12volts/(max speed in rev/sec) Typical Falcon 6000revs/min or 100 revs/sec
                .withKA(0.01)  // "arbitrary" amount to provide crisp response
                .withKG(0)     // gravity can be used for elevator or arm
                .withKP(6)     // 2 revs yields 12 volts
                .withKI(0)
                .withKD(0.1);
        Robot.configureMotor(armExtendMotor, cfg);
    }

    private void configClawMotor() {
        TalonFXConfiguration clawCfg = new TalonFXConfiguration();
        clawCfg.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake);
        Robot.configureMotor(clawMotor, clawCfg);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rotations", armExtendMotor.getPosition().getValue());
        SmartDashboard.putNumber("Arm Velocity", armExtendMotor.getVelocity().getValue());
        distExtender();
    }

    public void distExtender(){
        Pose2d currentPose = rc.drivetrain.getState().Pose;
        Translation2d target = Robot.convertTran(new Translation2d(1.36,5.55));
        double distance = currentPose.getTranslation().minus(target).getNorm();
        distance *= (20/2);
        distance = MathUtil.clamp(distance, 0, 20);
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

    // private class HomingStateMachine extends MMStateMachine {
    // private boolean hasHomed = false;

    // // Start, Safety, Speed, MoveToHome, Home, Normal
    // public boolean hasHomed() {
    // return hasHomed;
    // }

    // public void reset() {
    // hasHomed = false;
    // setInitial(sStart);
    // }

    // public HomingStateMachine() {
    // reset();
    // }

    // MMStateMachineState sStart = new MMStateMachineState("sStart") {

    // @Override
    // public MMStateMachineState calcNextState() {
    // return sSafety;
    // }
    // };

    // MMStateMachineState sSafety = new MMStateMachineState("sSafety") {

    // @Override
    // public MMStateMachineState calcNextState() {
    // return this;
    // if (hasHomed) {
    // return sSpeed;
    // }
    // if (rC.intakeSubsystem.getArmExtend() < startUpPosition -
    // Constants.Arm.Extend.safetyDistance) {
    // nextState = HomeStates.Speed;
    // }
    // if (rC.intakeSubsystem.getCloseToHomeSensor()
    // || ((rC.intakeSubsystem.getArmExtend() <
    // Constants.Arm.Extend.safetyDistance)
    // && hasHomed)) {
    // nextState = HomeStates.MoveToHome;
    // }
    // if (rC.intakeSubsystem.getHomeSensor()) {
    // nextState = HomeStates.Home;
    // }
    // }

    // @Override
    // public void transitionTo(MMStateMachineState previousState) {
    // rC.intakeSubsystem.setHomeSlow();
    // startUpPosition = rC.intakeSubsystem.getArmExtend();
    // }

    // };
    // }

}
