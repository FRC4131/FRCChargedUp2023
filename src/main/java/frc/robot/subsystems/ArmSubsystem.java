// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

  // 5:1, 4:1, 3:1 (gearboxes) + 3.86:1 (15-tooth to 58-tooth output gears) gear
  // ratios ratio
  private final double ARM_MOTOR_GEAR_RATIO = 5.0 / 1.0 * 4.0 / 1.0 * 3.0 / 1.0 * 58.0 / 15.0;

  // private CANSparkMax m_actuator = new CANSparkMax(0, MotorType.kBrushless);
  private TalonSRX m_actuator = new TalonSRX(30);
  // private CANSparkMax m_leftRot = new CANSparkMax(59, MotorType.kBrushless);
  private CANSparkMax m_rightRot = new CANSparkMax(58, MotorType.kBrushless);

  private double desiredAngle;
  private double desiredDistance;

  private SparkMaxPIDController m_rightRotPIDController;
  private SparkMaxPIDController m_actuatorPIDController;

  private RelativeEncoder m_rightEncoder;

  private boolean bool;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // m_actuator.configFeed
    m_rightEncoder = m_rightRot.getEncoder();
    m_rightRotPIDController = m_rightRot.getPIDController();
    m_rightEncoder.setPositionConversionFactor(1);
    m_rightRotPIDController.setOutputRange(-1, 1);
    m_rightRotPIDController.setSmartMotionMaxAccel(7500, 0);
    m_rightRotPIDController.setSmartMotionMaxVelocity( 10000, 0);
    m_rightRotPIDController.setSmartMotionMinOutputVelocity(0, 0);
    m_rightRotPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_rightRotPIDController.setP(4e-9);
    m_rightRotPIDController.setI(0);
    m_rightRotPIDController.setD(0);
    m_rightRotPIDController.setFF(9.9e-5);
    SmartDashboard.putBoolean("bool", bool);
    // m_leftRot.setIdleMode(IdleMode.kCoast);

    // m_leftRot.fp
    resetPosition();

    // m_actuatorPIDController = m_actuator.getPIDController();

    // m_leftRot.follow(m_rightRot, true);

    m_rightRot.burnFlash();
    m_rightRotPIDController.setReference(0, ControlType.kDutyCycle);
  }

  public void rotateArm(DoubleSupplier powerSupplier) {
    // m_rightRot.set(powerSupplier.getAsDouble() * 0.1);
    m_rightRotPIDController.setReference(powerSupplier.getAsDouble() * 0.5, ControlType.kDutyCycle);
    SmartDashboard.putNumber("TOSE", powerSupplier.getAsDouble());
  }

  public void snapToAngle(double angleDegrees) {
    double encoderRotations = angleToMotorRotations(angleDegrees);
    m_rightRotPIDController.setReference(encoderRotations, ControlType.kSmartMotion);
  };

  public void resetPosition() {
    m_rightEncoder.setPosition(0);
    // m_rightRotPIDController.setReference(0, ControlType.kPosition);
  }

  private double angleToMotorRotations(double angleDegrees) {
    return angleDegrees * ARM_MOTOR_GEAR_RATIO / 360.0;
  }

  public void extendTo(double length) {
    m_actuatorPIDController.setReference(lengthToUnits(length), ControlType.kPosition);
  }

  

  private double lengthToUnits(double length) {
    // TODO: THIS
    return 0;
  }

  public void extendArm(double power) {
    m_actuator.set(TalonSRXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("RightencoderRpos", m_rightEncoder.getPosition() / ARM_MOTOR_GEAR_RATIO * 360 + " ticks(?)");
    // SmartDashboard.putString("LeftencoderRpos", m_leftEncoder.getPosition() + "
    // ticks(?)");
    SmartDashboard.getBoolean("BOOL", false);
  }

  public CommandBase resetEncoder() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          resetPosition();
        });
  }
}
