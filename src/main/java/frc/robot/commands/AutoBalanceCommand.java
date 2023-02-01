// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
DrivetrainSubsystem m_drivetrainSubsystem;
PoseEstimationSubsystem m_poseEstimationSubsystem;
PIDController pitchPIDController;

  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    pitchPIDController = new PIDController(0.25, 0, 0);
    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pitchPIDController.setSetpoint(-2.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double threshold = 0.125;
    double driveSignal = pitchPIDController.calculate(m_poseEstimationSubsystem.getPitch());
    

    if (Math.abs(driveSignal) > threshold) {
      if (driveSignal > 0.0) {
        driveSignal = threshold;
      } else {
        driveSignal = -threshold;
      }
    }
    SmartDashboard.putNumber("DriveSignal", driveSignal);

    
    m_drivetrainSubsystem.drive(new Translation2d(0, driveSignal * 2) , 0, m_poseEstimationSubsystem.getPose().getRotation(), true, false);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(0, 0) , 1, m_poseEstimationSubsystem.getPose().getRotation(), true, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
