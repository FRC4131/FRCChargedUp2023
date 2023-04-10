// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  VisionSubsystemLL3 m_visionSubsystem;
  private final Field2d field2d = new Field2d();
  static SwerveDrivePoseEstimator m_swerveDrivePoseEst;
  AHRS m_navX;

  public frc.lib.util.SwerveModule[] mSwerveMods;

  /** Creates a new PoseEstimationSubsystem. */
  public PoseEstimationSubsystem(DrivetrainSubsystem drivetrainSubsystem, VisionSubsystemLL3 visionSubsystem) {
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_visionSubsystem = visionSubsystem;

    m_navX = new AHRS(SPI.Port.kMXP, (byte) 200);

    zeroGyro();

    m_swerveDrivePoseEst = new SwerveDrivePoseEstimator(
        Constants.Swerve.swerveKinematics,
        getGyroYaw(),
        m_drivetrainSubsystem.getModulePositions(),
        new Pose2d()
        ,VecBuilder.fill(0.05, 0.05, 0.05),
        VecBuilder.fill(0.6, 0.6, 100.0)
        );
    
        
  }

  public void zeroGyro() {
    m_navX.zeroYaw();
  }

  public void setAngleAdjustment(double adjustment){
    m_navX.setAngleAdjustment(adjustment);
  }

  private Rotation2d getGyroYaw() {
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - m_navX.getYaw())
        : Rotation2d.fromDegrees(m_navX.getYaw());
  }

  public void resetOdometry(Pose2d pose) {
    m_swerveDrivePoseEst.resetPosition(getGyroYaw(), m_drivetrainSubsystem.getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return m_swerveDrivePoseEst.getEstimatedPosition();
  }

  public double getPitch() {
    return m_navX.getPitch();
  }

  public double getRoll() {
    return m_navX.getRoll();
  }

  public double getYaw() {
    return m_navX.getYaw();
  }

  public double[] getTargetTagData(){
    return m_visionSubsystem.getTargetTagData();
  }

  @Override
  public void periodic() {
    EstimatedRobotPose aprilTagPose = m_visionSubsystem.getAprilTagRobotPose().orElse(null);
    DriverStation.refreshData();
    if (aprilTagPose != null && (!DriverStation.isAutonomous())) {
      m_swerveDrivePoseEst.addVisionMeasurement(aprilTagPose.estimatedPose.toPose2d(), aprilTagPose.timestampSeconds);
    }
    m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions());
    field2d.setRobotPose(m_swerveDrivePoseEst.update(getGyroYaw(), m_drivetrainSubsystem.getModulePositions()));
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("RawGyroYaw", getGyroYaw().getDegrees());
    SmartDashboard.putNumber("x", m_swerveDrivePoseEst.getEstimatedPosition().getX());
    SmartDashboard.putNumber("y", m_swerveDrivePoseEst.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Odom Rotation", m_swerveDrivePoseEst.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Robot Pitch", getPitch());
    SmartDashboard.putNumber("Robot Roll", getRoll());
    SmartDashboard.putNumber("Robot Yaw", getYaw());

    SmartDashboard.putNumber("Mi Distanco Por Favor", m_visionSubsystem.getTargetTagData()[0]);
    
    SmartDashboard.putData(field2d);

  }

}
