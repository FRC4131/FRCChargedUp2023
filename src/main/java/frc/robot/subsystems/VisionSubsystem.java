// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AprilTagConstants;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera m_camera;
  PhotonPoseEstimator m_photonPoseEstimator;
  double yaw;
  double pitch;
  double area;
  double skew;
  boolean hasTargets;
  PhotonPipelineResult result;
  Transform3d m_cameraToRobot;
  Optional<EstimatedRobotPose> m_estimatedRobotPose;

  List<AprilTag> tagList = new ArrayList<>(8);
  AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tagList, 16.54175,8.0137);

  public VisionSubsystem()
  {
    m_camera = new PhotonCamera("4131Camera0");
    m_cameraToRobot = new Transform3d(new Translation3d(0.3302, 0.0, 0.1778), new Rotation3d(0.0, 0.0, 0.0));
    tagList.add(AprilTagConstants.tag1);
    tagList.add(AprilTagConstants.tag2);
    tagList.add(AprilTagConstants.tag3);
    tagList.add(AprilTagConstants.tag4);
    tagList.add(AprilTagConstants.tag5);
    tagList.add(AprilTagConstants.tag6);
    tagList.add(AprilTagConstants.tag7);
    tagList.add(AprilTagConstants.tag8);
    
    m_photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_camera, m_cameraToRobot);
  }

  public Optional<EstimatedRobotPose> getAprilTagRobotPose()
  {
    return m_estimatedRobotPose;
  }

  @Override
  public void periodic() {
    m_estimatedRobotPose = m_photonPoseEstimator.update();
    // SmartDashboard.putNumber("Distance to Target", seek());
    // SmartDashboard.putBoolean("hasTargets", result.hasTargets());
    // if(hasTargets){
    //   SmartDashboard.putNumber("Target ID", targetID);
    //   SmartDashboard.putNumber("Yaw", yaw);
    //   SmartDashboard.putNumber("Pitch", pitch);
    //   SmartDashboard.putNumber("Area", area);
    //   SmartDashboard.putNumber("Skew", skew);
    // }
  }

}
