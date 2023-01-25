// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class GoToPoseCommand extends CommandBase {
  /** Creates a new PoseCommand. */
  
  DrivetrainSubsystem m_DrivetrainSubsystem;
  PoseEstimationSubsystem m_PoseEstimationSubsystem;
  Pose2d m_setpointPose;
  PIDController x_Controller;
  PIDController y_Controller;
  PIDController theta_Controller;

  public GoToPoseCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem, Pose2d pose) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_PoseEstimationSubsystem = poseEstimationSubsystem;
    m_setpointPose = pose;

    x_Controller = new PIDController(1, 0, 0);
    y_Controller = new PIDController(1, 0, 0);
    theta_Controller = new PIDController(4.0, 0, 0);

    theta_Controller.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(m_DrivetrainSubsystem, m_PoseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_Controller.setSetpoint(m_setpointPose.getX());
    y_Controller.setSetpoint(m_setpointPose.getY());
    theta_Controller.setSetpoint(m_setpointPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredRotation = theta_Controller.calculate(m_PoseEstimationSubsystem.getPose().getRotation().getRadians());
    double desiredX = x_Controller.calculate(m_PoseEstimationSubsystem.getPose().getX());
    double desiredY = y_Controller.calculate(m_PoseEstimationSubsystem.getPose().getY());
    m_DrivetrainSubsystem.drive(new Translation2d(desiredX, desiredY), desiredRotation, m_PoseEstimationSubsystem.getPose().getRotation(), false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
