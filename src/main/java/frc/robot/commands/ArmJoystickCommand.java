// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.System.Logger.Level;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmJoystickCommand extends CommandBase {

  private final ArmSubsystem m_ArmSubsystem;
  private final DoubleSupplier m_rotationDoubleSupplier;



  /** Creates a new ArmRotateCommand. */
  public ArmJoystickCommand(ArmSubsystem armSubsys, DoubleSupplier rightJoy) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_ArmSubsystem = armSubsys;
    m_rotationDoubleSupplier = rightJoy;

    addRequirements(armSubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ArmSubsystem.resetPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_ArmSubsystem.snapToAngle(5);
    m_ArmSubsystem.rotateArm(m_rotationDoubleSupplier.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ArmSubsystem.rotateArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
