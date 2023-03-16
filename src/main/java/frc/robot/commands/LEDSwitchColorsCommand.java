// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDSwitchColorsCommand extends CommandBase {
    private final LEDSubsystem m_LEDSubsystem;

    /** Creates a new LEDSwitchColorsCommand. */
    public LEDSwitchColorsCommand(LEDSubsystem ledSubsystem) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_LEDSubsystem = ledSubsystem;
        addRequirements(ledSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if (m_LEDSubsystem.getH() == 65 && m_LEDSubsystem.getS() == 255 && m_LEDSubsystem.getV() == 255) {
            m_LEDSubsystem.setHSV(-1, 130, 255, 255);
        } else if (m_LEDSubsystem.getH() == 130 && m_LEDSubsystem.getS() == 255 && m_LEDSubsystem.getV() == 255) {
            m_LEDSubsystem.setHSV(-1, 65, 255, 255);
        } else {
            m_LEDSubsystem.setHSV(-1, 65, 255, 255);
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}