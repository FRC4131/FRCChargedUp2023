// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class CalibrateOdometryCommand extends CommandBase {
  /** Creates a new CalibrateOdometryCommand. */
  private final PoseEstimationSubsystem m_poseEstimationSubsystem;
  private final Pose2d m_pose;
  public CalibrateOdometryCommand(PoseEstimationSubsystem poseEstimationSubsystem, Pose2d pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_pose = pose;
    addRequirements(poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_poseEstimationSubsystem.resetOdometry(m_pose);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
