// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.photonvision.RobotPoseEstimator;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera m_camera;
  RobotPoseEstimator m_photonPoseEstimator;
  double yaw;
  double pitch;
  double area;
  double skew;
  boolean hasTargets;
  PhotonPipelineResult result;

  public VisionSubsystem()
  {
    m_camera = new PhotonCamera("Limelight1");
    Transform3d m_cameraToRobot = new Transform3d(new Translation3d(0.3302, 0.0, 0.1778), new Rotation3d(0.0, 0.0, 0.0));
    // m_photonPoseEstimator = new RobotPoseEstimator(null, null, null, )
  }

  @Override
  public void periodic() {
    var result = m_camera.getLatestResult(); 
    boolean hasTargets = result.hasTargets();
    if(!hasTargets){
      return;
    }
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();

    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();
    skew = target.getSkew();
    
    
    SmartDashboard.putNumber("Distance to Target", seek());
    SmartDashboard.putBoolean("hasTargets", result.hasTargets());
    if(hasTargets){
      SmartDashboard.putNumber("Target ID", targetID);
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Skew", skew);
    }
  }

  public double seek(){
    if(hasTargets){
    double range =
    PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.CAMERA_HEIGHT_METERS,
            VisionConstants.TARGET_HEIGHT_METERS,
            VisionConstants.CAMERA_PITCH_RADIANS,
            Units.degreesToRadians(getPitch()));

    return range;
  } else return 0.0;
}
  public double getPitch(){
    var new_result = result.getBestTarget().getPitch();
    return new_result;
  }
}
