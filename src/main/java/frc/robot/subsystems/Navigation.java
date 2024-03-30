// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Objects;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.ejml.simple.SimpleMatrix;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.MMUtilities.MMField;

public class Navigation extends SubsystemBase {
  RobotContainer rc;
  public static int visionUpdate = 0;
  private boolean hasNoteTarget;
  private double noteX;
  private double noteY;
  // private LimelightTarget_Detector[] leftLimelightDetector;
  private String limelightFrontName = "limelight-front";
  private String limelightBackUpName = "limelight-backup";
  private String limelightBackDownName = "limelight-bd";
  public boolean useVision = true;
  public boolean oneTargetBack = false;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable backUpLimelight = inst.getTable(limelightBackUpName);
  private final NetworkTable backDownLimelight = inst.getTable(limelightBackDownName);
  private final NetworkTable frontLimelight = inst.getTable(limelightFrontName);

  StructPublisher<Pose2d> backLimelightPose = NetworkTableInstance.getDefault()
      .getStructTopic("backLimelightPose4", Pose2d.struct).publish();
  StructPublisher<Pose2d> frontLimelightPose = NetworkTableInstance.getDefault()
      .getStructTopic("frontLimelightPose4", Pose2d.struct).publish();
  StructPublisher<Pose2d> predictedPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("predictedPosePublisher", Pose2d.struct).publish();

  BooleanLogEntry useVisionBoolean;
  DoubleLogEntry visionCounterLog;
  BooleanLogEntry updatedVisionFront;
  BooleanLogEntry updatedVisionBack;
  StringLogEntry xVelocityLog;
  StringLogEntry yVelocityLog;
  StringLogEntry angVelocityLog;
  StringLogEntry predictedPoseLog;
  StringLogEntry actualPoseLog;

  Pigeon2 pigeon;
  double hitAcceleration;
  private double llFrontHeartBeat = 0;
  private double llBackUpHeartBeat = 0;

  private ArrayList<Double> xVelocity = new ArrayList<Double>();
  private ArrayList<Double> yVelocity = new ArrayList<Double>();
  private ArrayList<Double> angVelocity = new ArrayList<Double>();
  public static Pose2d predictedPose;
  private int predictionCycles = 25;
  double timeToShoot = .1;
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
    pigeon = rc.drivetrain.getPigeon2();
    // aprilThread = new AprilThread();
    // aprilThread.start();
    DataLog log = DataLogManager.getLog();
    visionCounterLog = new DoubleLogEntry(log, "my/vision/counter");
    updatedVisionBack = new BooleanLogEntry(log, "my/vision/updateBack");
    updatedVisionFront = new BooleanLogEntry(log, "my/vision/updateFront");
    xVelocityLog = new StringLogEntry(log, "my/otf/xVelocity");
    yVelocityLog = new StringLogEntry(log, "my/otf/yVelocity");
    angVelocityLog = new StringLogEntry(log, "my/otf/angVelocity");
    predictedPoseLog = new StringLogEntry(log, "my/otf/predictedPose");
    actualPoseLog = new StringLogEntry(log, "my/otf/actualPose");

  }

  @Override
  public void periodic() {
    Pose2d pose = rc.drivetrain.getState().Pose;
    currentPose = pose;
    visionCounterLog.append(visionUpdate);
    updatedVisionBack.append(false);
    updatedVisionFront.append(false);

    if (Math.abs(pigeon.getAccelerationX().getValue()) > hitAcceleration
        || Math.abs(pigeon.getAccelerationY().getValue()) > hitAcceleration) {
      resetVision();
    }

    if (pose.getX() < 0 || pose.getX() > 16.54 ||
        pose.getY() < 0 || pose.getY() > 8.23) {
      resetVision();
    }
   
    boolean fieldBoundsFilter = (currentPose.getX() < 4.8 || currentPose.getX() > 11.6);

    if (useVision) {// || fieldBoundsFilter
      double currentHB = backUpLimelight.getEntry("hb").getDouble(0);
      boolean hasBackUpTarget = backUpLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;

      if (currentHB != llBackUpHeartBeat) {
        llBackUpHeartBeat = currentHB;

        double[] def = new double[] { 0, 0 };

        double[] bp = backUpLimelight.getEntry("botpose_wpiblue").getDoubleArray(def);

        double[] tp = backUpLimelight.getEntry("targetpose_robotspace").getDoubleArray(def);
        if (bp.length > 7 && tp.length > 5) {

          double numberOfTargets = bp[7];
          double distanceToTarget = tp[1];

          boolean robotOnFloor = Math.abs(bp[2]) <= 1;
          boolean closeToTag = distanceToTarget <= 1.6;

          // if (hasBackUpTarget && ((numberOfTargets >= 1 && oneTargetBack)
          // || numberOfTargets > 1 || visionUpdate < 50)) {
          if (hasBackUpTarget && robotOnFloor && closeToTag) {
            Pose2d llPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
            backLimelightPose.accept(llPose);
            SmartDashboard.putString("llBackUpPose", llPose.toString());
            // double margin = pose.minus(llPose).getTranslation().getNorm();
            // double margin = 0;
            // if (visionUpdate < 50
            // || margin < .25
            // || (((numberOfTargets >= 1 && oneTargetBack)
            // || numberOfTargets > 1) && margin < .75)) {
            if (true) {
              // double[] stdDevArray = { .9, .9, .9 };
              // SimpleMatrix stdDevs = new SimpleMatrix(stdDevArray);
              // rc.drivetrain.setVisionMeasurementStdDevs(new Matrix(stdDevs));
              double totalLatency = bp[6];
              // double latency_capture = backUpLimelight.getEntry("cl").getDouble(0);
              // double latency_pipeline = backUpLimelight.getEntry("tl").getDouble(0);
              // rc.drivetrain.setVisionMeasurementStdDevs(
              //     VecBuilder.fill(.5, .5, Units.degreesToRadians(6)));
              rc.drivetrain.addVisionMeasurement(llPose, 
                  Timer.getFPGATimestamp() - (totalLatency / 1000.0));
              visionUpdate++;
              updatedVisionBack.append(true);

            }
          }
        }
      }
    }

    if (useVision) {// fieldFilter
      double currentHB = frontLimelight.getEntry("hb").getDouble(0);
      boolean hasFrontTarget = frontLimelight.getEntry("tv").getNumber(0).doubleValue() > 0.5;

      if (currentHB != llFrontHeartBeat) {
        llFrontHeartBeat = currentHB;
        double[] def = new double[] { 0, 0, 0, 0, 0, 0 };
        double[] bp = frontLimelight.getEntry("botpose_wpiblue").getDoubleArray(def);
        if (bp.length > 7) {
          double numberOfTargets = bp[7];
          boolean robotOnFloor = Math.abs(bp[2]) <= 1;

          // if (hasFrontTarget && numberOfTargets > 1) {
          if (hasFrontTarget && robotOnFloor) {

            Pose2d llPose = new Pose2d(bp[0], bp[1], Rotation2d.fromDegrees(bp[5]));
            frontLimelightPose.accept(llPose);
            SmartDashboard.putString("llFrontPose", llPose.toString());
            // double margin = pose.minus(llPose).getTranslation().getNorm();
            // double margin = 0;
            // if (visionUpdate < 50
            // || margin < .25
            // || (numberOfTargets > 1 && margin < .75)) {
            if (true) {
              // double latency_capture = frontLimelight.getEntry("cl").getDouble(0);
              // double latency_pipeline = frontLimelight.getEntry("tl").getDouble(0);
              double totalLatency = bp[6];
              // double[] stdDevArray = { .7, .7, .9 };
              // SimpleMatrix stdDevs = new SimpleMatrix(stdDevArray);
              // rc.drivetrain.setVisionMeasurementStdDevs(new Matrix(stdDevs));
              // rc.drivetrain.setVisionMeasurementStdDevs(
              //     VecBuilder.fill(.5, .5, Units.degreesToRadians(6)));
              rc.drivetrain.addVisionMeasurement(llPose, 
                  Timer.getFPGATimestamp() - (totalLatency / 1000.0));
              visionUpdate++;
              updatedVisionFront.append(true);
            }
          }
        }
      }
    }

    rc.field.setRobotPose(pose);
    SmartDashboard.putString("MMpose", pose.toString());

    hasNoteTarget = false;
    double tv = backDownLimelight.getEntry("tv").getDouble(0);
    if (tv > 0) {
      hasNoteTarget = true;
      noteX = backDownLimelight.getEntry("tx").getDouble(0);
      noteY = backDownLimelight.getEntry("ty").getDouble(0);
    }
    SmartDashboard.putBoolean("NOTE TV", hasNoteTarget);
    SmartDashboard.putNumber("NOTE TX", noteX);
    SmartDashboard.putNumber("NOTE TY", noteY);
    ChassisSpeeds chassisSpeeds = rc.drivetrain.getCurrentRobotChassisSpeeds();
    updatePredictedPosition(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
    predictedPosePublisher.accept(predictedPose);
    xVelocityLog.append(xVelocity.toString());
    yVelocityLog.append(yVelocity.toString());
    angVelocityLog.append(angVelocity.toString());
    predictedPoseLog.append(predictedPose.toString());
    actualPoseLog.append(currentPose.toString());
  }

  public double getNoteX() {
    return noteX;
  }

  public double getNoteY() {
    return noteY;
  }

  public boolean noteBelowThreshold() {
    return noteY < 8;
  }

  public boolean hasNoteTarget() {
    return hasNoteTarget;
  }

  public String autoLeftOrRight() {
    if (noteX > 300) {
      return "NoteRight";
    } else {
      return "NoteStraight";
    }
  }

  public double getDistanceToSpeaker() {
    Pose2d currentPose = rc.drivetrain.getState().Pose;
    Translation2d target = MMField.currentSpeakerPose().getTranslation();
    double distance = currentPose.getTranslation().minus(target).getNorm();
    // if (rc.shooterSubsystem.targetAngleSpeaker != null) {
    // distance -= Math.sin(rc.shooterSubsystem.targetAngleSpeaker.getRadians());
    // }
    return distance;

  }

  public void setAutoVisionOn(boolean isUpdate) {
    useVision = isUpdate;
  }

  public void setOneTargetBack(boolean isOneTarget) {
    oneTargetBack = isOneTarget;
  }

  public double getPredictedDistanceToSpeaker() {
    if (predictedPose != null) {
      Translation2d target = MMField.currentSpeakerPose().getTranslation();
      double distance = predictedPose.getTranslation().minus(target).getNorm();
      return distance;
    }

    return 0;
  }

  public void updatePredictedPosition(double x, double y, double r) {
    xVelocity.add(x);
    yVelocity.add(y);
    angVelocity.add(r);
    limitArrayLists();
    syncPredictedPosition();
  }

  public void syncPredictedPosition() {
    double xVelocitySum = 0;
    double yVelocitySum = 0;
    double angVelocitySum = 0;
    for (int i = 0; i < xVelocity.size(); i++) {
      xVelocitySum += xVelocity.get(i);
    }
    for (int i = 0; i < yVelocity.size(); i++) {
      yVelocitySum += yVelocity.get(i);
    }
    for (int i = 0; i < angVelocity.size(); i++) {
      angVelocitySum += angVelocity.get(i);
    }
    double averageX = (xVelocitySum / xVelocity.size()) * timeToShoot;
    double averageY = (yVelocitySum / yVelocity.size()) * timeToShoot;
    double averageAng = (angVelocitySum / angVelocity.size()) * timeToShoot;

    predictedPose = currentPose
        .plus(new Transform2d(new Translation2d(averageX, averageY), new Rotation2d(averageAng)));
  }

  public void limitArrayLists() {
    if (xVelocity.size() > predictionCycles) {
      xVelocity.remove(0);
    }
    if (yVelocity.size() > predictionCycles) {
      yVelocity.remove(0);
    }
    if (angVelocity.size() > predictionCycles) {
      angVelocity.remove(0);
    }
  }

  public void resetVision() {
    visionUpdate = 0;
    // aprilThread.ResetVisionUpdate();
  }

  public void updateLog() {

  }

  public Navigation setFrontLimelightPipeline(int pipeline) {
    frontLimelight.getEntry("pipeline").setNumber(pipeline);
    return this;
  }

  public Navigation setBackUpLimelightPipeline(int pipeline) {
    backUpLimelight.getEntry("pipeline").setNumber(pipeline);
    return this;
  }

  public Navigation setBackDownLimelightPipeline(int pipeline) {
    backDownLimelight.getEntry("pipeline").setNumber(pipeline);
    return this;
  }

}
