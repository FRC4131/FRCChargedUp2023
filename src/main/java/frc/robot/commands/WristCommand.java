// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristCommand extends CommandBase {
  /** Creates a new WristCommand. */
  private final WristSubsystem m_WristSubsystem;
  boolean isClockwise;

  /**
   * Backwards sorry
   * @param wristSubsystem
   * @param isClockwise true for neo to point upward when arm is stowed
   */
  public WristCommand(WristSubsystem wristSubsystem, boolean isClockwise) {
    m_WristSubsystem = wristSubsystem;
    this.isClockwise = isClockwise;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (!isClockwise) {
      m_WristSubsystem.rotateCounterClockwise();
    } else {
      m_WristSubsystem.rotateClockwise();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isClockwise) {
      m_WristSubsystem.rotateCounterClockwise();
    } else {
      m_WristSubsystem.rotateClockwise();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_WristSubsystem.stopRotate();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!isClockwise && m_WristSubsystem.getClockwiseSwitch()) {
      return true;
    } else if (isClockwise && m_WristSubsystem.getCounterClockwiseSwitch()) {
      return true;
    } else
      return false;
  }
}
