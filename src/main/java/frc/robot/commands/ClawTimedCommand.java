// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawTimedCommand extends CommandBase {
  /** Creates a new ClawTimedCommand. */
  ClawSubsystem m_clawSubsystem;
  Timer m_timer;
  double m_seconds;
  double m_speed;
  public ClawTimedCommand(ClawSubsystem clawSubsystem, double seconds, double speed) {
    m_clawSubsystem = clawSubsystem;
    m_timer = new Timer();
    m_speed = speed; 
    addRequirements(clawSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_clawSubsystem.intakeSpeed(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_clawSubsystem.intakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_seconds);
  }
}
