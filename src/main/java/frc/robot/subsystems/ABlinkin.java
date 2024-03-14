// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class ABlinkin extends SubsystemBase {
  Spark sparkblinkin = new Spark(0);
  RobotContainer rc;
  public double intakeBlinkValue = -.07;
  public double gottemBlinkValue = -.77;
  public double shootBlinkValue = -.17;
  public double gimmeNoteValue = -.68;
  public double normalValue = 0;
  public double errorValue = .61;

  /** Creates a new spark. */
  public ABlinkin(RobotContainer rc) {
    this.rc = rc;
  }

  @Override
  public void periodic() {

  }

  public void controlBlink(double value) {
    sparkblinkin.set(value);
  }

  public void gimmeNote(){
    sparkblinkin.set(gimmeNoteValue);
  }

  public void gotNote() {
    sparkblinkin.set(normalValue);
  }

  public void error() {
        sparkblinkin.set(errorValue);

  }
}
