// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class NewAutoBalanceCommand extends CommandBase {

  DrivetrainSubsystem m_DrivetrainSubsystem;
  PoseEstimationSubsystem m_PoseEstimationSubsystem;
  Translation2d offset;
  Translation2d startOffset;

  public NewAutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimationSubsystem poseEstimationSubsystem) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_PoseEstimationSubsystem = poseEstimationSubsystem;
    addRequirements(drivetrainSubsystem, poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startOffset = new Translation2d(m_PoseEstimationSubsystem.getRoll(), -m_PoseEstimationSubsystem.getPitch());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    offset = new Translation2d(m_PoseEstimationSubsystem.getRoll(), -m_PoseEstimationSubsystem.getPitch());

    if (!(startOffset.getNorm() - offset.getNorm() > startOffset.getNorm() / 7.5)) {
      if (Math.abs(offset.getNorm()) <= 12)
        offset.times(0.3);
      if (Math.abs(offset.getNorm()) <= 10)
        offset.times(0.125);
      if (Math.abs(offset.getNorm()) <= 8)
        offset.times(0.08);
      if (Math.abs(offset.getNorm()) <= 5)
        offset.times(0);
      m_DrivetrainSubsystem.drive(offset.times(0.045), 0.0,
          m_PoseEstimationSubsystem.getPose().getRotation(), false,
          false);
    } else {
      m_DrivetrainSubsystem.drive(new Translation2d(), 0.1,
          m_PoseEstimationSubsystem.getPose().getRotation(), false,
          false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(0, 0),
        0.1,
        m_PoseEstimationSubsystem.getPose().getRotation(),
        false,
        false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}