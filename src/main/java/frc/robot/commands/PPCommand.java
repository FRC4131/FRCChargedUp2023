// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class PPCommand extends CommandBase {
  private final PathPlannerTrajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final DrivetrainSubsystem m_drivetrainSubsystem;
  private final PoseEstimationSubsystem m_poseEstimationSubsystem;
  private final Timer m_timer = new Timer();
  private HolonomicDriveController m_controller;
  private PIDController m_xController;
  private PIDController m_yController;
  private ProfiledPIDController m_thetaController;

  /** Creates a new PPCommand. */
  public PPCommand(DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimationSubsystem poseEstimationSubsystem,
      PathPlannerTrajectory trajectory) {
    // Use addRequirements() here to declare subsystem dependencies

    if (DriverStation.getAlliance().equals(Alliance.Blue)){
      m_trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, Alliance.Blue);
    }
    else{
      m_trajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(trajectory, Alliance.Red);
    }
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_pose = m_poseEstimationSubsystem::getPose;

    addRequirements(drivetrainSubsystem, poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_poseEstimationSubsystem.resetOdometry(m_trajectory.getInitialHolonomicPose());

    // m_poseEstimationSubsystem.setAdjustment(DriverStation.getAlliance().equals(Alliance.Blue)
    // ? 0 : 180);
    m_xController = new PIDController(3.5, 0, 0);
    m_yController = new PIDController(3.5, 0, 0);
    m_thetaController = new ProfiledPIDController(6, 0, 0,
        new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 4));
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // Reset PID controller error and set initial pose.
    m_xController.reset();
    m_yController.reset();
    m_thetaController.reset(m_poseEstimationSubsystem.getPose().getRotation().getRadians(), 0.0);

    // Add PID controllers to the drive controller
    m_controller = new HolonomicDriveController(m_xController, m_yController, m_thetaController);
    m_controller.setEnabled(true);
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate our desired chassis speeds to follow the trajectory using our PID
    // and drive controllers
    double curTime = m_timer.get();
    var desiredState = (PathPlannerState) m_trajectory.sample(curTime);
    var targetChassisSpeeds = m_controller.calculate(m_pose.get(), desiredState, desiredState.holonomicRotation);

    var updatedTargetChassisSpeeds = new ChassisSpeeds(targetChassisSpeeds.vxMetersPerSecond,
        targetChassisSpeeds.vyMetersPerSecond, targetChassisSpeeds.omegaRadiansPerSecond);

    m_drivetrainSubsystem.drive(updatedTargetChassisSpeeds,
        Math.abs(desiredState.velocityMetersPerSecond), Math.abs(desiredState.angularVelocityRadPerSec));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
    m_drivetrainSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }
}
