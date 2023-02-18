// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.TargetingSubsystem;

public class GoToPoseTeleopCommand extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem;
  PoseEstimationSubsystem m_poseEstimationSubsystem;
  TargetingSubsystem m_targetingSubsystem;
  DoubleSupplier m_x;
  DoubleSupplier m_y;
  DoubleSupplier m_theta;
  DoubleSupplier m_throttle;
  Pose2d m_desiredPose;
  double m_minThrottle = 0.2;

  PIDController m_xController;
  PIDController m_yController;
  PIDController m_thetaController;

  /** Creates a new GoToPoseTeleopCommand. */
  public GoToPoseTeleopCommand(DrivetrainSubsystem drivetrainSubsystem, 
    PoseEstimationSubsystem poseEstimationSubsystem,
    TargetingSubsystem targetingSubsystem, 
    DoubleSupplier x, 
    DoubleSupplier y,
    DoubleSupplier theta, 
    DoubleSupplier throttle) {
      m_drivetrainSubsystem = drivetrainSubsystem;
      m_poseEstimationSubsystem = poseEstimationSubsystem;
      m_targetingSubsystem = targetingSubsystem;
      m_x = x;
      m_y = y;
      m_theta = theta;
      m_throttle = throttle;
      
      m_xController = new PIDController(3, 0, 0);
      m_yController = new PIDController(3, 0, 0);
      
      m_thetaController = new PIDController(4.0, 0, 0);
      m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

      addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem, m_targetingSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_desiredPose = m_targetingSubsystem.getTargetGridPose();
    m_xController.setSetpoint(m_desiredPose.getX());
    m_yController.setSetpoint(m_desiredPose.getY());
    m_thetaController.setSetpoint(m_desiredPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidDesiredRotation = m_thetaController.calculate(m_poseEstimationSubsystem.getPose().getRotation().getRadians());
    double pidDesiredX = m_xController.calculate(m_poseEstimationSubsystem.getPose().getX());
    double pidDesiredY = m_yController.calculate(m_poseEstimationSubsystem.getPose().getY());

    //Cap the PID's velocity vector by the max speed 
    Translation2d pidVector = new Translation2d(pidDesiredX, pidDesiredY);
    double pidScaling = getSpeedScaling(pidVector);
    Translation2d scaledPidVector = pidVector.times(pidScaling);

    //Get the driver input vector rotated into the global coordinate system
    Translation2d rotatedDriveVector = new Translation2d(m_x.getAsDouble(), m_y.getAsDouble()).rotateBy(m_poseEstimationSubsystem.getPose().getRotation());

    //Calc the throttle scaling factor
    double throttleSlope = 1 - m_minThrottle;
    double throttleScale = throttleSlope * m_throttle.getAsDouble() + m_minThrottle;

    Translation2d finalVector = new Translation2d(
                                        (rotatedDriveVector.getX() + scaledPidVector.getX()) * throttleScale,
                                        (rotatedDriveVector.getY() + scaledPidVector.getY()) * throttleScale);
    double finalRotation = (m_theta.getAsDouble() + pidDesiredRotation) * throttleScale;

    m_drivetrainSubsystem.drive(finalVector, finalRotation, new Rotation2d(), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getSpeedScaling(Translation2d input)
  {
    if(input.getNorm() > Constants.Swerve.maxSpeed)
    {
      return Constants.Swerve.maxSpeed / input.getNorm();
    }
    else
    {
      return 1.0;
    }
  }

}
