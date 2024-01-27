// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMConfigure;
import frc.robot.MMUtilities.MMField;
import frc.robot.MMUtilities.MMFiringSolution;
import frc.robot.MMUtilities.MMTurnPIDController;
import frc.robot.MMUtilities.MMWaypoint;

public class Shooter extends SubsystemBase {
  RobotContainer rc;
  MMFiringSolution firingSolution = new MMFiringSolution(
      new MMWaypoint(1.3, -.111, 50),
      new MMWaypoint(2.5, -.078, 70),
      new MMWaypoint(3.97, -.057, 70));
  private Pose2d speakerPose;
  private MMTurnPIDController turnPidController = new MMTurnPIDController();
  private Pose2d currentPose;
  private double distanceToSpeaker;
  private double speakerTurnRate;
  private Rotation2d targetAngleSpeaker;

  private MMWaypoint desiredWaypoint;

  TalonFX shooterRotateMotor = new TalonFX(12, "CANIVORE");
  CANcoder shooterRotateCanCoder = new CANcoder(5, "CANIVORE");

  private final PositionVoltage shooterRotationPosition = new PositionVoltage(0);
  private final MotionMagicVoltage shooterRotateMotionMagicVoltage = new MotionMagicVoltage(0);

  /** Creates a new Shooter. */
  public Shooter(RobotContainer rc) {
    this.rc = rc;
    configShooterRotateCanCoder();
    configShooterRotateMotor();
  }

  private void configShooterRotateCanCoder() {
    CANcoderConfiguration canConfig = new CANcoderConfiguration();
    canConfig.MagnetSensor
        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Signed_PlusMinusHalf)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withMagnetOffset(-0.390869140625);
    MMConfigure.configureDevice(shooterRotateCanCoder, canConfig);
  }

  private void configShooterRotateMotor() {
    double cruiseVelocity = .5; // Sensor revolutions/second
    double timeToReachCruiseVelocity = .4; // seconds
    double timeToReachMaxAcceleration = .2; // seconds

    double maxSupplyVoltage = 12; // Max supply
    double staticFrictionVoltage = 1; //
    double rotorToSensorRatio = (60.0 / 15.0) * 48.0;
    double maxRotorVelocity = 100.0; // Max speed for Falcon500 100 rev/sec
    double maxSensorVelocity = maxRotorVelocity / rotorToSensorRatio; // Max speed in sensor units/sec
    double feedForwardVoltage = (maxSupplyVoltage - staticFrictionVoltage) / maxSensorVelocity; // Full Voltage/Max
                                                                                                // Sensor Velocity

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
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(12)
        .withKI(0)
        .withKD(2);
    cfg.Feedback
        .withFeedbackRemoteSensorID(shooterRotateCanCoder.getDeviceID())
        .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
        .withSensorToMechanismRatio(1)
        .withRotorToSensorRatio(rotorToSensorRatio);
    cfg.Slot1
        .withKS(1) // voltage to overcome static friction
        .withKV(0)
        .withKA(0) // "arbitrary" amount to provide crisp response
        .withKG(0) // gravity can be used for elevator or arm
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(96 * 2)// 12
        .withKI(0)
        .withKD(.25);// 2
    MMConfigure.configureDevice(shooterRotateMotor, cfg);
  }

  @Override
  public void periodic() {
    speakerPose = MMField.currentSpeakerPose();
    currentPose = rc.drivetrain.getState().Pose;

    Translation2d transformFromSpeaker = speakerPose.getTranslation().minus(currentPose.getTranslation());
    targetAngleSpeaker = transformFromSpeaker.getAngle();

    turnPidController.initialize(targetAngleSpeaker);

    speakerTurnRate = turnPidController.execute(currentPose.getRotation());
    distanceToSpeaker = transformFromSpeaker.getNorm();

    desiredWaypoint = firingSolution.calcSolution(distanceToSpeaker);

    SmartDashboard.putNumber("Velocity", desiredWaypoint.getVelocity());
    SmartDashboard.putNumber("Distance To Target", distanceToSpeaker);
    SmartDashboard.putNumber("SpeakerTurnRate", speakerTurnRate);
    SmartDashboard.putString("Transform From target", transformFromSpeaker.toString());
    SmartDashboard.putNumber("Shooter Encoder Value", shooterRotateCanCoder.getAbsolutePosition().getValue());
    SmartDashboard.putNumber("AADesired Shooter Angle", desiredWaypoint.getAngle());
    SmartDashboard.putNumber("AAcurrent Shooter Angle", getCurrentShooterAngle());
    SmartDashboard.putNumber("AADesired Robot angle", targetAngleSpeaker.getDegrees());
    SmartDashboard.putNumber("AAcurrent Robot Angle", getCurrentRobotAngle().getDegrees());
  }

  public void shooterRotationRot(double rotations) {
    shooterRotateMotor.setControl(shooterRotateMotionMagicVoltage.withPosition(rotations));
  }

  public void shooterRotationPID(double rotations) {
    shooterRotateMotor.setControl(shooterRotationPosition.withSlot(1).withPosition(rotations));
  }

  public double getCurrentShooterAngle() {
    return shooterRotateMotor.getPosition().getValue();
  }

  public double getSpeakerTurnRate() {
    return speakerTurnRate;
  }

  public MMWaypoint getDesiredSpeakerWaypoint() {
    return desiredWaypoint;
  }

  public Rotation2d getTargetAngleSpeaker() {
    return targetAngleSpeaker;
  }

  public Rotation2d getCurrentRobotAngle() {
    return currentPose.getRotation();
  }

  public boolean isSpeakerShotReady(){
    //double desiredShooterAngle = getDesiredSpeakerWaypoint().getAngle();
    //double currentShooterAngle = getCurrentShooterAngle();
    //Rotation2d desiredRobotAngle = getTargetAngleSpeaker();
    //Rotation2d currentRobotAngle = getCurrentRobotAngle();
    
    return(Math.abs(getCurrentRobotAngle().minus(getTargetAngleSpeaker()).getDegrees()) < 2 
    && Math.abs(getCurrentShooterAngle() - getDesiredSpeakerWaypoint().getAngle()) < .01 );
  }
}
