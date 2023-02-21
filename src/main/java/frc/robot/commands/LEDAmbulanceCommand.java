// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDAmbulanceCommand extends CommandBase {
  private final LEDSubsystem m_LEDSubsystem;
  private final Timer timer = new Timer();


  /** Creates a new LEDFredCommand. */
  public LEDAmbulanceCommand(LEDSubsystem ledSubsystem) {
    m_LEDSubsystem = ledSubsystem;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LEDSubsystem.setHSV(-1, 0, 100, 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_LEDSubsystem.setHSV(-1, 212, 89, 93);
      Timer.delay(2);
      m_LEDSubsystem.setHSV(-1, 0, 100, 100);
      Timer.delay(2);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(15.0);
  }
}
