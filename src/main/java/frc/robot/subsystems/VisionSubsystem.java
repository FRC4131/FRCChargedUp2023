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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Limelight1");
  public PhotonPipelineResult result;
  public double yaw;
  public double pitch;
  public double area;
  public double skew;
  public boolean hasTargets;
  

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    SmartDashboard.putNumber("Distance to Target", -1);
    SmartDashboard.putBoolean("hasTargets", result.hasTargets());
    if (!hasTargets){ 
      return;}
    
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();

    yaw = target.getYaw();
    pitch = target.getPitch();
    area = target.getArea();
    skew = target.getSkew();

    double dist = PhotonUtils.calculateDistanceToTargetMeters(
        VisionConstants.CAMERA_HEIGHT_METERS,
        VisionConstants.TARGET_HEIGHT_METERS,
        VisionConstants.CAMERA_PITCH_RADIANS,
        Units.degreesToRadians(pitch));

    SmartDashboard.putNumber("Distance to Target", -dist);
    SmartDashboard.putNumber("Target ID", targetID);
    SmartDashboard.putNumber("Yaw", yaw);
    SmartDashboard.putNumber("Pitch", pitch);
    SmartDashboard.putNumber("Area", area);
    SmartDashboard.putNumber("Skew", skew);


  }

}
