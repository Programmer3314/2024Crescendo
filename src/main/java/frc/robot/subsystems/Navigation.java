// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Detector;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class Navigation extends SubsystemBase {
  RobotContainer rc;
  public static int visionUpdate = 0;
  private boolean hasLeftNoteTarget;
  private double leftNoteX;
  private double leftNoteY;
  private LimelightTarget_Detector[] leftLimelightDetector;
  private String limelightFrontName = "limelight-front";
  private String limelightBackUpName = "limelight-backup";
  public boolean useVision = true;

  Pigeon2 pigeon;
  double hitAcceleration;
  private double llHeartBeat = 0;
  // private ArrayList<Double> xVelocity = new ArrayList<Double>();
  // private ArrayList<Double> yVelocity = new ArrayList<Double>();
  // private ArrayList<Double> angVelocity = new ArrayList<Double>();
  public static Pose2d predictedPose;
  // private int predictionCycles;
  // double timeToShoot = .1;
  private Pose2d currentPose;

  // private AprilThread aprilThread;
  // protected final ReadWriteLock aprilStateLock = new ReentrantReadWriteLock();

  // // TODO: Try parsing limelights in a thread...
  // // I read that the json parsing is very time consuming
  // public class AprilThread {
  // double llFrontHearbeat = 0;
  // double llBackHeartbeat = 0;
  // long visionUpdate = 0;

  // protected static final int START_THREAD_PRIORITY = 1;
  // protected final Thread thread;
  // protected volatile boolean running;
  // protected final double UpdateFrequency = 25;

  // public AprilThread() {
  // thread = new Thread(this::run);
  // /*
  // * Mark this thread as a "daemon" (background) thread
  // * so it doesn't hold up program shutdown
  // */
  // thread.setDaemon(true);
  // }

  // /**
  // * Starts the odometry thread.
  // */
  // public void start() {
  // running = true;
  // thread.start();
  // }

  // /**
  // * Stops the odometry thread.
  // */
  // public void stop() {
  // stop(0);
  // }

  // /**
  // * Stops the odometry thread with a timeout.
  // *
  // * @param millis The time to wait in milliseconds
  // */
  // public void stop(long millis) {
  // running = false;
  // try {
  // thread.join(millis);
  // } catch (final InterruptedException ex) {
  // Thread.currentThread().interrupt();
  // }
  // }

  // public void run() {
  // Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);
  // while (running) {
  // try {
  // aprilStateLock.writeLock().lock();

  // Timer.delay(1.0 / UpdateFrequency);
  // Pose2d pose = rc.drivetrain.getState().Pose;
  // {
  // var lastResult =
  // LimelightHelpers.getLatestResults(limelightBackUpName).targetingResults;
  // // SmartDashboard.putNumber("LL Heartbeat",
  // // lastResult.timestamp_LIMELIGHT_publish);

  // if (lastResult.timestamp_LIMELIGHT_publish != llBackHeartbeat) {
  // llBackHeartbeat = lastResult.timestamp_LIMELIGHT_publish;

  // if (lastResult.valid && lastResult.targets_Fiducials.length > 1) {
  // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
  // // SmartDashboard.putString("llPose", llPose.toString());
  // double margin = pose.minus(llPose).getTranslation().getNorm();
  // // double margin = 0;
  // if (visionUpdate < 50
  // || margin < .25
  // || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
  // rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp()
  // - (lastResult.latency_capture / 1000.0) - (lastResult.latency_pipeline /
  // 1000.0));
  // visionUpdate++;
  // }
  // }
  // }
  // }
  // {
  // var lastResult =
  // LimelightHelpers.getLatestResults(limelightFrontName).targetingResults;
  // // SmartDashboard.putNumber("LL Heartbeat",
  // // lastResult.timestamp_LIMELIGHT_publish);

  // if (lastResult.timestamp_LIMELIGHT_publish != llFrontHearbeat) {
  // llHeartBeat = lastResult.timestamp_LIMELIGHT_publish;

  // if (lastResult.valid && lastResult.targets_Fiducials.length > 1) {
  // Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
  // // SmartDashboard.putString("llPose", llPose.toString());
  // double margin = pose.minus(llPose).getTranslation().getNorm();
  // // double margin = 0;
  // if (visionUpdate < 50
  // || margin < .25
  // || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
  // rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp()
  // - (lastResult.latency_capture / 1000.0) - (lastResult.latency_pipeline /
  // 1000.0));
  // visionUpdate++;
  // }
  // }
  // }
  // }
  // } finally {
  // aprilStateLock.writeLock().unlock();
  // }
  // }
  // }

  // public void ResetVisionUpdate() {
  // try {
  // aprilStateLock.writeLock().lock();
  // visionUpdate = 0;
  // } finally {
  // aprilStateLock.writeLock().unlock();
  // }
  // }
  // }

  /** Creates a new Navigation. */
  public Navigation(RobotContainer rc) {
    this.rc = rc;
    SmartDashboard.putData("FieldX", rc.field);
    LimelightHelpers.setPipelineIndex("limelight-bd", 0);
    LimelightHelpers.setPipelineIndex("limelight-backup", 0);
    LimelightHelpers.setPipelineIndex("limelight-front", 0);
    pigeon = rc.drivetrain.getPigeon2();
    // aprilThread = new AprilThread();
    // aprilThread.start();

  }

  @Override
  public void periodic() {
    Pose2d pose = rc.drivetrain.getState().Pose;
    currentPose = pose;

    if (Math.abs(pigeon.getAccelerationX().getValue()) > hitAcceleration
        || Math.abs(pigeon.getAccelerationY().getValue()) > hitAcceleration) {
      resetVision();
    }

    if (pose.getX() < 0 || pose.getX() > 16.54 ||
        pose.getY() < 0 || pose.getY() > 8.23) {
      resetVision();
    }
    // TODO: try a couple of latency approaches
    // a.
    // Timer.getFPGATimestamp() - (result.latency_capture / 1000.0) -
    // (result.latency_pipeline / 1000.0)
    // and maybe also subtract latency_jsonParse/1000.0
    // b.
    // LimelightHelpers.PoseEstimate limelightMeasurement =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // if(limelightMeasurement.tagCount >= 2)
    // {
    // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
    // m_poseEstimator.addVisionMeasurement(
    // limelightMeasurement.pose,
    // limelightMeasurement.timestampSeconds);
    // }

    if (useVision || (currentPose.getX() < 4.8 || currentPose.getX() > 11.6)) {
      var lastResult = LimelightHelpers.getLatestResults(limelightBackUpName).targetingResults;
      SmartDashboard.putNumber("LL Heartbeat", lastResult.timestamp_LIMELIGHT_publish);

      if (lastResult.timestamp_LIMELIGHT_publish != llHeartBeat) {
        llHeartBeat = lastResult.timestamp_LIMELIGHT_publish;

        if (lastResult.valid && lastResult.targets_Fiducials.length > 1) {
          Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
          SmartDashboard.putString("llPose", llPose.toString());
          // double margin = pose.minus(llPose).getTranslation().getNorm();
          double margin = 0;
          if (visionUpdate < 50
              || margin < .25
              || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
            // rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp()
                - (lastResult.latency_capture / 1000.0) - (lastResult.latency_pipeline / 1000.0)
                - (lastResult.latency_jsonParse / 1000.0));
            visionUpdate++;
          }
        }
      }
    }

    if (useVision || (currentPose.getX() < 4.8 || currentPose.getX() > 11.6)) {
      var lastResult = LimelightHelpers.getLatestResults(limelightFrontName).targetingResults;
      SmartDashboard.putNumber("LL Heartbeat", lastResult.timestamp_LIMELIGHT_publish);

      if (lastResult.timestamp_LIMELIGHT_publish != llHeartBeat) {
        llHeartBeat = lastResult.timestamp_LIMELIGHT_publish;

        if (lastResult.valid && lastResult.targets_Fiducials.length > 1) {
          Pose2d llPose = lastResult.getBotPose2d_wpiBlue();
          SmartDashboard.putString("llPose", llPose.toString());
          // double margin = pose.minus(llPose).getTranslation().getNorm();
          double margin = 0;
          if (visionUpdate < 50
              || margin < .25
              || (lastResult.targets_Fiducials.length > 1 && margin < 1)) {
            // rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp());
            rc.drivetrain.addVisionMeasurement(llPose, Timer.getFPGATimestamp()
                - (lastResult.latency_capture / 1000.0) - (lastResult.latency_pipeline / 1000.0)
                - (lastResult.latency_jsonParse / 1000.0));
            visionUpdate++;
          }
        }
      }
    }

    rc.field.setRobotPose(pose);
    SmartDashboard.putString("MMpose", pose.toString());

    // var llpython = LimelightHelpers.getPythonScriptData("limelight-bd");
    // hasLeftNoteTarget = llpython[0] > .5;
    // leftNoteX = llpython[1];
    // leftNoteY = llpython[2];
    hasLeftNoteTarget = false;
    leftLimelightDetector = LimelightHelpers.getLatestResults("limelight-bd").targetingResults.targets_Detector;
    if (leftLimelightDetector.length > 0) {
      hasLeftNoteTarget = true;
      leftNoteX = leftLimelightDetector[0].tx_pixels;
      leftNoteY = leftLimelightDetector[0].ty_pixels;
    }

    SmartDashboard.putNumber("TX:", leftNoteX);
    SmartDashboard.putNumber("TY", leftNoteY);
  }

  public double getLeftNoteX() {
    return leftNoteX;
  }

  // public double getScaledLeftX(){
  // double scaleValue = 1;
  // if(leftNoteY>400){
  // scaleValue = 4;
  // }
  // else if(leftNoteY>300){
  // scaleValue = 3;
  // }
  // else if(leftNoteY>200){
  // scaleValue = 2;
  // }
  // return leftNoteX/
  // }

  public double getLeftNoteY() {
    return leftNoteY;
  }

  public boolean hasLeftNoteTarget() {
    return hasLeftNoteTarget;
  }

  public String autoLeftOrRight() {
    if (leftNoteX > 300) {
      return "NoteRight";
    } else {
      return "NoteStraight";
    }
  }

  public double getDistanceToSpeaker() {
    Pose2d currentPose = rc.drivetrain.getState().Pose;
    Translation2d target = MMField.currentSpeakerPose().getTranslation();
    // double distance = currentPose.getTranslation().minus(target).getX();
    double distance = currentPose.getTranslation().minus(target).getNorm();
    // if (rc.shooterSubsystem.targetAngleSpeaker != null) {
    // distance -= Math.sin(rc.shooterSubsystem.targetAngleSpeaker.getRadians());
    // }
    return distance;
  }

  public void setAutoVisionOn(boolean isUpdate) {
    useVision = isUpdate;
  }
  // public double getPredictedDistanceToSpeaker() {
  // Pose2d currentPose = predictedPose;
  // Translation2d target = MMField.currentSpeakerPose().getTranslation();
  // double distance = currentPose.getTranslation().minus(target).getNorm();
  // return distance;
  // }

  // public void updatePredictedPosition(double x, double y, double r) {
  // xVelocity.add(x);
  // yVelocity.add(y);
  // angVelocity.add(r);
  // limitArrayLists();
  // syncPredictedPosition();
  // }

  // public void syncPredictedPosition() {
  // double xVelocitySum = 0;
  // double yVelocitySum = 0;
  // double angVelocitySum = 0;
  // for (int i = 0; i < xVelocity.size(); i++) {
  // xVelocitySum += xVelocity.get(i);
  // }
  // for (int i = 0; i < yVelocity.size(); i++) {
  // yVelocitySum += yVelocity.get(i);
  // }
  // for (int i = 0; i < angVelocity.size(); i++) {
  // angVelocitySum += angVelocity.get(i);
  // }
  // double averageX = xVelocitySum / xVelocity.size();
  // double averageY = yVelocitySum / yVelocity.size();
  // double averageAng = angVelocitySum / angVelocity.size();

  // predictedPose = currentPose.plus(new Transform2d())
  // }

  // public void limitArrayLists() {
  // if (xVelocity.size() > predictionCycles) {
  // xVelocity.remove(xVelocity.size() - 1);
  // }
  // if (yVelocity.size() > predictionCycles) {
  // yVelocity.remove(yVelocity.size() - 1);
  // }
  // if (angVelocity.size() > predictionCycles) {
  // angVelocity.remove(angVelocity.size() - 1);
  // }
  // }
  public void resetVision() {
    visionUpdate = 0;
    // aprilThread.ResetVisionUpdate();
  }
}
