// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TargetingSubsystem;

public class ArmPositionCommand extends CommandBase {
  /** Creates a new ArmPositionCommand. */
  ArmSubsystem m_armSubsystem;
  TargetingSubsystem m_targetingSubsystem;
  public ArmPositionCommand(ArmSubsystem armSubsystem, TargetingSubsystem targetingSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_armSubsystem = armSubsystem;
    m_targetingSubsystem = targetingSubsystem;
    addRequirements(m_armSubsystem, m_targetingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_armSubsystem.snapToAngle(m_targetingSubsystem.getScoringHeight().rotation);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
