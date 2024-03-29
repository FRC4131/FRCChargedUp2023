// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class AutoBalanceCommand extends CommandBase {
  /** Creates a new AutoBalanceCommand. */
  private DrivetrainSubsystem m_drivetrainSubsystem;
  private PoseEstimationSubsystem m_poseEstimationSubsystem;
  private PIDController pitchPIDController;
  private double balancedAngleDegrees = 0;
  private boolean isRed;
  private boolean alignRot;
  private PIDController yawPIDController;
  private double OFFSET = 0;
  private double flipPitch;

  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
      boolean alignRot) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.alignRot = alignRot;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    // pitchPIDController = new PIDController(0.0002, 0, 0);
    pitchPIDController = new PIDController(0.0002, 0, 0);
    yawPIDController = new PIDController(0.0002, 0, 0);
    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);

    yawPIDController.enableContinuousInput(-Math.PI, Math.PI);
    flipPitch = 1;
  }
  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem, PoseEstimationSubsystem poseEstimationSubsystem,
      boolean alignRot, boolean flipPitch) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.alignRot = alignRot;
    m_drivetrainSubsystem = drivetrainSubsystem;
    m_poseEstimationSubsystem = poseEstimationSubsystem;
    // pitchPIDController = new PIDController(0.0002, 0, 0);
    pitchPIDController = new PIDController(0.0002, 0, 0);
    yawPIDController = new PIDController(0.0002, 0, 0);
    addRequirements(m_drivetrainSubsystem, m_poseEstimationSubsystem);

    yawPIDController.enableContinuousInput(-Math.PI, Math.PI);
    this.flipPitch = flipPitch ? -1 : 1;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DriverStation.refreshData();
    isRed = DriverStation.getAlliance().equals(Alliance.Red); 
    pitchPIDController.reset();
    pitchPIDController.setSetpoint(balancedAngleDegrees);
    pitchPIDController.setTolerance(2);
    yawPIDController.reset();
    yawPIDController.setTolerance(Math.toRadians(1.0));
    // yawPIDController.setSetpoint(isRed ? 180.0 : 0.0); //change 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double threshold = 0.7; // was 0.125
    if (Math.abs(m_poseEstimationSubsystem.getPitch()) + OFFSET > 6.5) {
      pitchPIDController.setP(0.045);
    } else {
      pitchPIDController.setP(0.0004);
    }
    double driveSignal = pitchPIDController.calculate(m_poseEstimationSubsystem.getPitch() + OFFSET);
    driveSignal = MathUtil.clamp(driveSignal, -threshold, threshold);

    // UNUSED
    double rotationSignal = yawPIDController.calculate(m_poseEstimationSubsystem.getYaw());

    // if (Math.abs(pitchPIDController.getPositionError()) > 9.5) {
    // driveSignal *= 1.5;
    // }
    if (Math.abs(pitchPIDController.getPositionError()) + OFFSET < 4) {
      driveSignal *= 0;
    }
    if (Math.abs(pitchPIDController.getPositionError()) + OFFSET < 12) {
    driveSignal *= 0.7;
    }
 
    SmartDashboard.putNumber("DriveSignal", driveSignal);

    driveSignal *= -1;
    driveSignal *= flipPitch;

    m_drivetrainSubsystem.drive(new Translation2d(driveSignal, 0), driveSignal/1.5,
        isRed ? m_poseEstimationSubsystem.getPose().getRotation().rotateBy(Rotation2d.fromDegrees(180)) //change
            : m_poseEstimationSubsystem.getPose().getRotation(),
        true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new Translation2d(0, 0), 1, m_poseEstimationSubsystem.getPose().getRotation(), true,
        false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
