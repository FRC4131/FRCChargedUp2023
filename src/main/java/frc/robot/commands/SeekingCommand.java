// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;



public class SeekingCommand extends CommandBase {
  private final VisionSubsystem m_VisionSubsystem;
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final  PoseEstimationSubsystem m_poseEstimationSubsys;
  private PIDController turnController = new PIDController(0, 0, 0); //needs to be tuned


  /** Creates a new SeekingCommand. */
  public SeekingCommand(VisionSubsystem visionSubsystem, DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseestmationsubsys) 
  {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_VisionSubsystem = visionSubsystem;
    m_poseEstimationSubsys = poseestmationsubsys;
    addRequirements(visionSubsystem, drivetrainSubsystem, poseestmationsubsys);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  public void seek(){
    double rotationalSpeed;
    // if(m_VisionSubsystem.hasTargets){
    //   rotationalSpeed = -turnController.calculate(m_VisionSubsystem.yaw, 0);
    // } else rotationalSpeed = 0;

    // m_DrivetrainSubsystem.drive(new Translation2d(), rotationalSpeed, m_poseEstimationSubsys.getPose().getRotation(),  true, true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    seek();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new Translation2d(), 0, m_poseEstimationSubsys.getPose().getRotation(), true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
