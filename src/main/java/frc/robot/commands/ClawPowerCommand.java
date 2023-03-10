// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClawSubsystem;

public class ClawPowerCommand extends CommandBase {

  ClawSubsystem m_claw;
  double direction;

  /** Creates a new ClawPowerCommand. Intaking is positive <p>
   *  Auto reduced to 60%. Use a magnitude of 5/3 to run at full speed.
  */
  public ClawPowerCommand(ClawSubsystem clawsub, double dir) {
    m_claw = clawsub;
    direction = dir;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(clawsub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.intakeSpeed(.6 * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_claw.intakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
