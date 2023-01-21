// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimationSubsys extends SubsystemBase {


  private DrivetrainSubsystem drivetrainSubsystem;
  private VisionSubsystem visionSubsystem;

  /** Creates a new PoseEstimationSubsys. */
  public PoseEstimationSubsys(DrivetrainSubsystem driveSubsys, VisionSubsystem visSubsys) {

    this.drivetrainSubsystem = driveSubsys;
    this.visionSubsystem = visSubsys;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


    var odometryPose = drivetrainSubsystem.getPose();
    //var photonPose = visionSubsystem.getPose();
  
  }
}
