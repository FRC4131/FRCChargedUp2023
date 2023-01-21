// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class TurnToAngleCommand extends CommandBase {
  /** Creates a new TurnToAngleCommand. */
  DrivetrainSubsystem m_drivetrainSubsystem;
  PoseEstimationSubsystem m_poseEstimationSubsystem;
  Double m_setPointAngle;

  ProfiledPIDController m_pidController;

  public TurnToAngleCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem, Double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    m_setPointAngle = angle;

    m_pidController = new ProfiledPIDController(0.4, 0, 0,
                            new TrapezoidProfile.Constraints(20 * 2 * Math.PI, 20 * 2 * Math.PI));
    m_pidController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_pidController.setGoal(m_setPointAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Double rotateVel = m_pidController.calculate(m_poseEstimationSubsystem.getYaw().getRadians());
    m_drivetrainSubsystem.drive(new Translation2d(), rotateVel, m_poseEstimationSubsystem.getYaw().getRadians(), false, true);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(), 0, new Rotation2d(), true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
