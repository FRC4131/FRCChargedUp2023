// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
  /** Creates a new DefaultDriveCommand. */
  private final DrivetrainSubsystem drivetrainSubsystem;

  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;
  private final DoubleSupplier m_rot;

  public DefaultDriveCommand(DrivetrainSubsystem subsystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
    drivetrainSubsystem = subsystem;
    m_x = x;
    m_y = y;
    m_rot = rot;
    addRequirements(drivetrainSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(m_x.getAsDouble(), m_y.getAsDouble(),
        m_rot.getAsDouble(), drivetrainSubsystem.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
