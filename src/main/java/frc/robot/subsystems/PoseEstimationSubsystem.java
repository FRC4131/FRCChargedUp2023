// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.robot.Constants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_DrivetrainSubsystem;
  VisionSubsystem m_VisionSubsystem;
  SwerveDriveOdometry m_SwerveDriveOdometry;
  AHRS m_navX;
  public frc.lib.util.SwerveModule[] mSwerveMods;

  /** Creates a new PoseEstimationSubsystem. */
  public PoseEstimationSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystem visionSubsystem) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_VisionSubsystem = visionSubsystem;
    m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    zeroGyro();

    m_SwerveDriveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), m_DrivetrainSubsystem.getModulePositions());
    

  }
  
  public void zeroGyro() {
    m_navX.reset();
}

public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_navX.getYaw())
            : Rotation2d.fromDegrees(m_navX.getYaw());
}
  public void resetOdometry(Pose2d pose) {
    m_SwerveDriveOdometry.resetPosition(getYaw(), m_DrivetrainSubsystem.getModulePositions(), pose);
    for (SwerveModule mod : mSwerveMods) {
        mod.reset();
    }
}
  
  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run


  }
}
