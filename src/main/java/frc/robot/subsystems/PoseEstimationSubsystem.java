// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
  }
  
  public Pose2d getPose() 
  {
    return m_swerveDrivePoseEst.getEstimatedPosition();
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose){
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public double getRoll() {
    return m_navX.getRoll();
  }

  /**
   * 
   * @return The current yaw angle of the robot in degrees (-180,180].
   */
  public double getYaw(){
    return getGyroYaw().getDegrees();
  }

  @Override
  public void periodic() 
  {
    EstimatedRobotPose aprilTagPose = m_visionSubsystem.getAprilTagRobotPose().orElse(null);
    if(aprilTagPose != null)
    {
      m_swerveDrivePoseEst.addVisionMeasurement(aprilTagPose.estimatedPose.toPose2d(), aprilTagPose.timestampSeconds);
    }
    m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions());

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawGyroYaw", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("x", m_swerveDrivePoseEst.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y", m_swerveDrivePoseEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odom Rotation", m_swerveDrivePoseEst.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Pitch", getPitch()); 
    
  }
}
