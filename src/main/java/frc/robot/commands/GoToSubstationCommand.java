// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class GoToSubstationCommand extends CommandBase {
  /** Creates a new GoToSubstationCommand. */
  private DrivetrainSubsystem m_DrivetrainSubsystem;
  private PoseEstimationSubsystem m_PoseEstimationSubsystem;
  private AprilTag m_aprilTag; 
  private ProfiledPIDController x_Controller;
  private ProfiledPIDController y_Controller;
  private ProfiledPIDController theta_Controller;
  private Timer m_Timer;
  private Pose2d m_setpointPose;
  
  public GoToSubstationCommand (DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem) 
  {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_PoseEstimationSubsystem = poseEstimationSubsystem;
    m_aprilTag = (DriverStation.getAlliance().equals(Alliance.Blue) ? Constants.AprilTagConstants.tag4 : Constants.AprilTagConstants.tag5);

    x_Controller = new ProfiledPIDController(3, 0, 0,
    new TrapezoidProfile.Constraints(Swerve.maxSpeed, 8));
    
    y_Controller = new ProfiledPIDController(3, 0, 0, 
    new TrapezoidProfile.Constraints(Swerve.maxSpeed, 8));
    
    theta_Controller = new ProfiledPIDController(6, 0, 0,
    new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 4));

    theta_Controller.enableContinuousInput(-Math.PI, Math.PI);

    m_Timer = new Timer();
    addRequirements(m_DrivetrainSubsystem, m_PoseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
    x_Controller.reset(m_PoseEstimationSubsystem.getPose().getX());
    y_Controller.reset(m_PoseEstimationSubsystem.getPose().getY());
    m_setpointPose = m_aprilTag.pose.toPose2d();
    x_Controller.setGoal(m_setpointPose.getX());
    y_Controller.setGoal(m_setpointPose.getY());
    theta_Controller.setGoal(m_setpointPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double desiredRotation = theta_Controller.calculate(m_PoseEstimationSubsystem.getPose().getRotation().getRadians());
    double desiredX = x_Controller.calculate(m_PoseEstimationSubsystem.getPose().getX());
    double desiredY = y_Controller.calculate(m_PoseEstimationSubsystem.getPose().getY());
    SmartDashboard.putNumber("ABOOMX", m_setpointPose.getX());
    SmartDashboard.putNumber("ABOOMY", m_setpointPose.getY());
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
    return (x_Controller.atGoal() && y_Controller.atGoal() && theta_Controller.atGoal()) || m_Timer.hasElapsed(6);
  }
}
