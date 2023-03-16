// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WristSubsystem;

public class WristSwitchCommand extends CommandBase {
  private final WristSubsystem m_WristSubsystem;
  private boolean movingClockwise;

  /**
   * Command to turn the wrist to the opposite orientation.
   * <p>
   * Will turn clockwise if wrist has not hit either extrema upon activation.
   * 
   * @param wristSubsystem
   */
  public WristSwitchCommand(WristSubsystem wristSubsystem) {
    m_WristSubsystem = wristSubsystem;
    addRequirements(wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    movingClockwise = false;
    if (m_WristSubsystem.getCounterClockwiseSwitch()) {
      movingClockwise = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (movingClockwise) {
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
    if (movingClockwise && m_WristSubsystem.getClockwiseSwitch()) {
      return true;
    } else if (!movingClockwise && m_WristSubsystem.getCounterClockwiseSwitch()) {
      return true;
    } else
      return false;
  }
}
