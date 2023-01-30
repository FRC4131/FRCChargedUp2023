// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.AprilTagConstants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  VisionSubsystem m_visionSubsystem;
  SwerveDrivePoseEstimator m_swerveDrivePoseEst;
  AHRS m_navX;
  PhotonPoseEstimator photonPoseEstimator;
  public frc.lib.util.SwerveModule[] mSwerveMods;

  /** Creates a new PoseEstimationSubsystem. */
  public PoseEstimationSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    List<AprilTag> tagList = new ArrayList<>(8);
    tagList.add(AprilTagConstants.tag1);
    tagList.add(AprilTagConstants.tag2);
    tagList.add(AprilTagConstants.tag3);
    tagList.add(AprilTagConstants.tag4);
    tagList.add(AprilTagConstants.tag5);
    tagList.add(AprilTagConstants.tag6);
    tagList.add(AprilTagConstants.tag7);
    tagList.add(AprilTagConstants.tag8);

    AprilTagFieldLayout fieldLayout = new AprilTagFieldLayout(tagList, 16.54175,8.0137);
    //might need to change the camera to robot pose
    photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, m_visionSubsystem.m_camera, m_visionSubsystem.m_cameraToRobot);

    zeroGyro();
    
    m_swerveDrivePoseEst = new SwerveDrivePoseEstimator(
                                    Constants.Swerve.swerveKinematics, 
                                    getGyroYaw(), 
                                    m_drivetrainSubsystem.getModulePositions(),
                                    new Pose2d()
                                  );
  }
  
  public void zeroGyro() 
  {
    m_navX.reset();
  }

  private Rotation2d getGyroYaw() 
  {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_navX.getYaw())
            : Rotation2d.fromDegrees(m_navX.getYaw());
  }
  
  public void resetOdometry(Pose2d pose) 
  {
    m_swerveDrivePoseEst.resetPosition(getGyroYaw(), m_drivetrainSubsystem.getModulePositions(), pose);
    // for (SwerveModule mod : mSwerveMods) {
    //     mod.reset();
    // }
  }
  
  public Pose2d getPose() 
  {
    return m_swerveDrivePoseEst.getEstimatedPosition();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  @Override
  public void periodic() 
  {
    //m_swerveDrivePoseEst.addVisionMeasurement(m_visionSubsystem.getPose(), m_visionSubsystem.getTimestamp());
    m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawGyroYaw", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("x", m_swerveDrivePoseEst.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y", m_swerveDrivePoseEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odom Rotation", m_swerveDrivePoseEst.getEstimatedPosition().getRotation().getDegrees()); 
    
  }
}
