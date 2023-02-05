// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetingSubsystem extends SubsystemBase {
  
  //Grid Pose numbers correspond to April Tag IDs
  //Blue Alliance Wall
  Pose2d gridPose6 = new Pose2d(new Translation2d(1.613, 4.411), new Rotation2d(Math.toRadians(-179.3)));
  Pose2d gridPose7 = new Pose2d(new Translation2d(1.613, 2.75), new Rotation2d(Math.toRadians(-179.3)));
  Pose2d gridPose8 = new Pose2d(new Translation2d(1.613, 1.08), new Rotation2d(Math.toRadians(-179.3)));

  //Red Alliance Wall
  Pose2d gridPose3 = new Pose2d(new Translation2d(14.92, 4.411), new Rotation2d());
  Pose2d gridPose2 = new Pose2d(new Translation2d(14.92, 2.75), new Rotation2d());
  Pose2d gridPose1 = new Pose2d(new Translation2d(14.92, 1.08), new Rotation2d());

  Pose2d gridPoses[] = {gridPose1, gridPose2, gridPose3, gridPose6, gridPose7, gridPose8};
  int desiredGrid;

  
  /** Creates a new TargettingSubsystem. */
  public TargetingSubsystem() {}

  public Pose2d getTargetGridPose(){
    return gridPoses[desiredGrid];
  }

  public void setGridPose(int inputGrid){
    desiredGrid = inputGrid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
