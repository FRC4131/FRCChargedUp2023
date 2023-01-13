// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DefaultDriveCommand extends CommandBase {
  DrivetrainSubsystem m_DrivetrainSubsystem;
  DoubleSupplier x;
  DoubleSupplier y;
  DoubleSupplier theta;
  boolean fieldRelative;

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem, DoubleSupplier x, DoubleSupplier y,
      DoubleSupplier theta, boolean fieldRelative) {
    m_DrivetrainSubsystem = drivetrainSubsystem;
    this.x = x;
    this.y = y;
    this.theta = theta;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DrivetrainSubsystem.drive(new Translation2d(x.getAsDouble(), y.getAsDouble()), theta.getAsDouble(), true,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
