// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera = new PhotonCamera("Limelight1");

  @Override
  public void periodic() {
    var result = camera.getLatestResult(); 
    boolean hasTargets = result.hasTargets();
    if(!hasTargets){
      return;
    }
    List<PhotonTrackedTarget> targets = result.getTargets();
    PhotonTrackedTarget target = result.getBestTarget();
    int targetID = target.getFiducialId();

    double yaw = target.getYaw();
    double pitch = target.getPitch();
    double area = target.getArea();
    double skew = target.getSkew();
    
    SmartDashboard.putBoolean("hasTargets", result.hasTargets());
    if(hasTargets){
      SmartDashboard.putNumber("Target ID", targetID);
      SmartDashboard.putNumber("Yaw", yaw);
      SmartDashboard.putNumber("Pitch", pitch);
      SmartDashboard.putNumber("Area", area);
      SmartDashboard.putNumber("Skew", skew);
    }
  }
}
