// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

public class StandardAutoInit extends Command {
   RobotContainer rc;
   Pose2d initialPose;
   int pipeLineFront=0;
   int pipeLineLeft=1;

  /** Creates a new StandardAutoInit. */
  public StandardAutoInit( RobotContainer rc,Pose2d initialPose) {
   this.rc = rc;
   this.initialPose = initialPose;
    // Use addRequirements() here to declare subsystem dependencies.
  }
  public StandardAutoInit setPipeLine(int pipeLineFront,int pipeLineLeft){
      this.pipeLineFront = pipeLineFront;
      this.pipeLineLeft = pipeLineLeft;
      return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight-left", pipeLineLeft);
    LimelightHelpers.setPipelineIndex("limelight-front", pipeLineFront);
    rc.drivetrain.seedFieldRelative(initialPose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return true;
  }
}
