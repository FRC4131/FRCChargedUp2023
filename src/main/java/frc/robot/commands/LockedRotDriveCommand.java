// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class LockedRotDriveCommand extends CommandBase {
  private final DrivetrainSubsystem m_DrivetrainSubsystem;
  private final PoseEstimationSubsystem m_PoseEstimationSubsystem;
  
  private final DoubleSupplier m_x;
  private final DoubleSupplier m_y;

  private final DoubleSupplier m_sin;
  private final DoubleSupplier m_cos;

  private final DoubleSupplier m_throttle;

  private final ProfiledPIDController m_anglePID;
  private double currAngle;

  /** Creates a new LockedRotDriveCommand.
   *  <p> This produces a command to allow control over strafing while locking the robot's angle to reflect the direction of the right Joystick.
   *  <p> Be sure to deadband the right joystick.
   */
  public LockedRotDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
      PoseEstimationSubsystem poseEstimationSubsystem,
      DoubleSupplier leftX,
      DoubleSupplier leftY,
      DoubleSupplier rightX,
      DoubleSupplier rightY,
      DoubleSupplier throttle) {

    m_DrivetrainSubsystem = drivetrainSubsystem;
    m_PoseEstimationSubsystem = poseEstimationSubsystem;

    m_x = leftX;
    m_y = leftY;

    m_sin = rightY;
    m_cos = rightX;

    m_throttle = throttle;

    // Construct the PID controller that will be used to calculate the rotational movement.
    m_anglePID = new ProfiledPIDController(6, 0, 0,
        new TrapezoidProfile.Constraints(Math.PI * 4, Math.PI * 4));
    m_anglePID.enableContinuousInput(-180, 180);
    // currAngle = m_PoseEstimationSubsystem.getYaw();
    currAngle = m_PoseEstimationSubsystem.getYaw();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_anglePID.reset(currAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Calculate a desired rotational speed by using a PID controller to compare the current robot angle to a desired angle
    // The desired angle is calculated from the axes of the right joystick, using arctan to find the angle of the position
    // in degrees with a range of (-180, 180)
    // currAngle = m_PoseEstimationSubsystem.getYaw();
    currAngle = m_PoseEstimationSubsystem.getYaw();
    double wishAngle = Math.toDegrees(Math.atan2(m_cos.getAsDouble(), m_sin.getAsDouble()));
    double thetaSpeed = m_anglePID.calculate(currAngle, wishAngle);

    double minThrottle = 0.2;
    double slope = 1 - minThrottle;
    double scale = slope * m_throttle.getAsDouble() + minThrottle;

    m_DrivetrainSubsystem.drive(new Translation2d(m_x.getAsDouble() * scale,
                                m_y.getAsDouble() * scale),
                                Math.abs(thetaSpeed) < 0.05 ? 0 : thetaSpeed,
                                Rotation2d.fromDegrees(m_PoseEstimationSubsystem.getYaw()),
                                true,
                                true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
