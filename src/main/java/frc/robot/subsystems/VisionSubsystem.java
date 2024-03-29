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
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AprilTagConstants;
import frc.robot.Constants.AprilTagConstantsBlue;
import frc.robot.Constants.AprilTagConstantsRed;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera m_camera;
  PhotonPoseEstimator m_photonPoseEstimator;
  public double yaw;
  double pitch;
  double area;
  double skew;
  public boolean hasTargets;
  PhotonPipelineResult result;
  Transform3d m_cameraToRobot;
  Optional<EstimatedRobotPose> m_estimatedRobotPose;

  List<AprilTag> tagList = new ArrayList<>(8);
  AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    m_camera = new PhotonCamera("4131Camera0");
    m_cameraToRobot = new Transform3d(new Translation3d(Units.inchesToMeters(-11), 0.0, Units.inchesToMeters(6)),
        new Rotation3d(0.0, 0.0, Math.PI));

    DriverStation.refreshData();
    if (DriverStation.getAlliance().equals(Alliance.Red)) {
      tagList.add(AprilTagConstantsRed.tag1);
      tagList.add(AprilTagConstantsRed.tag2);
      tagList.add(AprilTagConstantsRed.tag3);
      tagList.add(AprilTagConstantsRed.tag5);
    }

    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      tagList.add(AprilTagConstantsBlue.tag4);
      tagList.add(AprilTagConstantsBlue.tag6);
      tagList.add(AprilTagConstantsBlue.tag7);
      tagList.add(AprilTagConstantsBlue.tag8);
    }
      fieldLayout = new AprilTagFieldLayout(tagList, 16.54175, 8.0137);
      m_photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_camera,
          m_cameraToRobot);
  }

  public Optional<EstimatedRobotPose> getAprilTagRobotPose() {
    return m_estimatedRobotPose;
  }

  public int getTagID() {
    return m_camera.getLatestResult().getBestTarget().getFiducialId();
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("Name", m_camera.getName());
    m_estimatedRobotPose = m_photonPoseEstimator.update();
    if (m_estimatedRobotPose.isPresent()) {
      SmartDashboard.putNumber("Apriltag X", m_estimatedRobotPose.get().estimatedPose.toPose2d().getX());
      SmartDashboard.putNumber("Apriltag Y", m_estimatedRobotPose.get().estimatedPose.toPose2d().getY());
      SmartDashboard.putNumber("Apriltag Heading (degrees)",
          m_estimatedRobotPose.get().estimatedPose.toPose2d().getRotation().getDegrees());
    }
  }

}
