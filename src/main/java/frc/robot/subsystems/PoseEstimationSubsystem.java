// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  VisionSubsystem m_visionSubsystem;
  SwerveDriveOdometry m_swerveDriveOdometry;
  AHRS m_navX;
  public frc.lib.util.SwerveModule[] mSwerveMods;

  /** Creates a new PoseEstimationSubsystem. */
  public PoseEstimationSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;
    m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    zeroGyro();

    m_swerveDriveOdometry = new SwerveDriveOdometry(
                                    Constants.Swerve.swerveKinematics, 
                                    getYaw(), 
                                    m_drivetrainSubsystem.getModulePositions()
                                  );
  }
  
  public void zeroGyro() 
  {
    m_navX.reset();
  }

  public Rotation2d getYaw() 
  {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_navX.getYaw())
            : Rotation2d.fromDegrees(m_navX.getYaw());
  }
  
  public void resetOdometry(Pose2d pose) 
  {
    m_swerveDriveOdometry.resetPosition(getYaw(), m_drivetrainSubsystem.getModulePositions(), pose);
    for (SwerveModule mod : mSwerveMods) {
        mod.reset();
    }
  }
  
  public Pose2d getPose() 
  {
    return m_swerveDriveOdometry.getPoseMeters();
  }

  @Override
  public void periodic() 
  {
    m_swerveDriveOdometry.update(getYaw(), m_drivetrainSubsystem.getModulePositions());
    
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("ROT VALUE", m_navX.getYaw());
        SmartDashboard.putNumber("x", m_swerveDriveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", m_swerveDriveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odom Rotation", m_swerveDriveOdometry.getPoseMeters().getRotation().getDegrees()); 
  }
}
