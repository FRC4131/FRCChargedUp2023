// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.*;

import org.photonvision.PhotonUtils;

public class SeekingCommand extends CommandBase {
  private final VisionSubsystem m_VisionSubsystem;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;


  /** Creates a new SeekingCommand. */
  public SeekingCommand(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_VisionSubsystem = visionSubsystem;
    addRequirements(visionSubsystem, drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
