// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystemLL3 extends SubsystemBase {
  Optional<EstimatedRobotPose> m_estimatedRobotPose;

  int numDetections = 10;
  int detectionThreshold = 9;
  ArrayList<Boolean> tagDetections = new ArrayList<Boolean>();

  public VisionSubsystemLL3() {}

  public Optional<EstimatedRobotPose> getAprilTagRobotPose() {
    return m_estimatedRobotPose;
  }

  public boolean IsAprilTagPersistent()
  {    
      int numValid = 0;
      for(int i=0; i < tagDetections.size(); i++)
      {
        if(tagDetections.get(i) == true)
        {
          numValid += 1;
        }
      }

      if(numValid > detectionThreshold)
      {
        return true;
      }
      else
      {
        return false;
      }
  }

  public Optional<EstimatedRobotPose> AprilTagUpdate()
  {
    double[] rawBotPose;
    Boolean validTargetsPresent = (1.0 == NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0));
    tagDetections.add(validTargetsPresent);
    if(tagDetections.size() > numDetections)
    {
      tagDetections.remove(0);
    }

    DriverStation.refreshData();
    int pipeline = 0;
    if(DriverStation.getAlliance().equals(Alliance.Blue))
    {
      pipeline = 1;
    }
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);

    if(validTargetsPresent)
    {
      if(DriverStation.getAlliance().equals(Alliance.Blue))
      {
        rawBotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
      }
      else
      {
        rawBotPose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpired").getDoubleArray(new double[6]);
      }
      
      return Optional.of(new EstimatedRobotPose(new Pose3d(new Translation3d(rawBotPose[0], rawBotPose[1], rawBotPose[2]), 
                                                new Rotation3d(rawBotPose[3], rawBotPose[4], rawBotPose[5])), 
                                                Timer.getFPGATimestamp() - (rawBotPose[6]/1000.0), 
                                                null));
    }
    else
    {
      return Optional.empty();
    }
  }

  @Override
  public void periodic() {
    m_estimatedRobotPose = AprilTagUpdate();
    SmartDashboard.putBoolean("IsAprilTagPersist", IsAprilTagPersistent());
    if (m_estimatedRobotPose.isPresent()) {
      SmartDashboard.putNumber("Apriltag X", m_estimatedRobotPose.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Apriltag Y", m_estimatedRobotPose.get().estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Apriltag Heading (degrees)",
          m_estimatedRobotPose.get().estimatedPose.toPose2d().getRotation().getDegrees());
    }
  }

}