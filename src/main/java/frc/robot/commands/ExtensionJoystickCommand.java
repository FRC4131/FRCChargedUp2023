// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtensionSubsystem;

public class ExtensionJoystickCommand extends CommandBase {
  /** Creates a new ExtensionCommand. */
  private final ExtensionSubsystem m_ExtensionSubsystem;
  private final DoubleSupplier m_extensionDoubleSupplier;
  public ExtensionJoystickCommand(ExtensionSubsystem extensionSubsystem, DoubleSupplier leftJoy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ExtensionSubsystem = extensionSubsystem;
    m_extensionDoubleSupplier = leftJoy;
    
    addRequirements(extensionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ExtensionSubsystem.extendArm(m_extensionDoubleSupplier.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ExtensionSubsystem.extendArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
