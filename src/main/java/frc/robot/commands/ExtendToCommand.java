// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtendToCommand extends CommandBase {
  private final ExtensionSubsystem m_ExtensionSubsystem;
  private Timer m_Timer;
  private double position;
  /** Creates a new ExtendToCommand. */
  public ExtendToCommand(ExtensionSubsystem extensionSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ExtensionSubsystem = extensionSubsystem;
    this.position = position;
    m_Timer = new Timer();
    addRequirements(extensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Timer.reset();
    m_Timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ExtensionSubsystem.extendTo(position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ExtensionSubsystem.extendArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ExtensionSubsystem.atGoal() || m_Timer.hasElapsed(5);
  }
}
