// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargettingSubsystem extends SubsystemBase {
  Pose2d gridPose = new Pose2d(new Translation2d(1.613, 4.411), new Rotation2d(Math.toRadians(-179.3)));
  
  /** Creates a new TargettingSubsystem. */
  public TargettingSubsystem() {}

  public Pose2d getTargetGridPose(){
    return gridPose;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
