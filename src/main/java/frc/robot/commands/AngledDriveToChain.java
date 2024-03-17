// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AngledDriveToChain extends Command {
  RobotContainer rc;
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  double distanceBetweenWheelsInches = 24;
  double driveTrainRadius = Math
      .sqrt((distanceBetweenWheelsInches / 2)*(distanceBetweenWheelsInches / 2) +(distanceBetweenWheelsInches / 2)*(distanceBetweenWheelsInches / 2));
  double driveTrainRadiusMeters = Units.inchesToMeters(driveTrainRadius);
  double driveRateRadPerSec = 1;// TODO: Alter this when adjusting turn rate
  double driveRateRadiansPerMeter =  driveRateRadPerSec/driveTrainRadiusMeters;
  double driveVector = (Math.sqrt(2) / 2) * driveRateRadiansPerMeter;

  public AngledDriveToChain(RobotContainer rc) {
    this.rc = rc;
    addRequirements(rc.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveRateY = rc.driverController.getLeftYSmoothed();
    double driveRateX = rc.driverController.getLeftXSmoothed();
    double turnRate = rc.driverController.getRightXSmoothed();
    // rc.drivetrain.setControl(drive
    //     .withVelocityX(-(Math.signum(driveRateY) * (Math.abs(driveRateY) + driveVector)))// negative for nick's
    //                                                                                      // preference, signum to retain
    //                                                                                      // the
    //     // sign of the driving, and abs to ensure the joystick and
    //     // calculated rate are going in the same direction
    //     .withVelocityY(-(Math.signum(driveRateX) * (Math.abs(driveRateX) + driveVector)))
    //     .withRotationalRate(turnRate));
    rc.drivetrain.setControl(drive
        .withVelocityX(0*(-Math.abs(turnRate)*driveVector))
        .withVelocityY(.6*(turnRate*driveVector))
        .withRotationalRate(1.5*turnRate));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
