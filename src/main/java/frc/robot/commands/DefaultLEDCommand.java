package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

// Based on old frc2022 defaultledcommand code

public class DefaultLEDCommand extends CommandBase {
    private final LEDSubsystem m_LEDSubsystem;

    public DefaultLEDCommand(LEDSubsystem ledSubsystem) {
        m_LEDSubsystem = ledSubsystem;

        addRequirements(ledSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("h", 0);
        SmartDashboard.putNumber("s", 0);
        SmartDashboard.putNumber("v", 0);
    }

    @Override
    public void execute() {
        int h = (int)SmartDashboard.getNumber("h", 0);
        int s = (int)SmartDashboard.getNumber("s", 0);
        int v = (int)SmartDashboard.getNumber("v", 0);

        m_LEDSubsystem.setHSV(-1, h, s, v);
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}