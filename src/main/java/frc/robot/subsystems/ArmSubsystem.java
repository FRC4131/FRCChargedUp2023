// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;




public class ArmSubsystem extends SubsystemBase {

  // private CANSparkMax m_actuator = new CANSparkMax(0, MotorType.kBrushless);
  private CANSparkMax m_leftRot = new CANSparkMax(21, MotorType.kBrushless);
  private CANSparkMax m_rightRot = new CANSparkMax(25, MotorType.kBrushless);

  private SparkMaxPIDController m_leftRotPIDController;
  private SparkMaxPIDController m_rightRotPIDController;
  private SparkMaxPIDController m_actuatorPIDController;

  private RelativeEncoder m_rightEncoder;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    m_rightEncoder = m_rightRot.getEncoder();
    m_rightRotPIDController = m_rightRot.getPIDController();
    m_rightRot.restoreFactoryDefaults();

    m_rightRotPIDController.setP(1);
    
    
    // m_actuatorPIDController = m_actuator.getPIDController();

    // m_leftRot.follow(m_rightRot, true);
  }

  public void rotateArm(DoubleSupplier powerSupplier){
    m_rightRot.set(powerSupplier.getAsDouble());
  }

  public void rotateTo(){
    m_rightRotPIDController.setReference(5.0, ControlType.kPosition);
  };

  public void resetPosition(){
    m_rightEncoder.setPosition(0);
    m_rightRotPIDController.setReference(0, ControlType.kPosition);
  }

  private double angleToUnits(double angle) {
    return 0;
    //TODO: THIS
  }

  public void extendTo(double length){
    m_actuatorPIDController.setReference(lengthToUnits(length),ControlType.kPosition);
  }
  private double lengthToUnits(double length) {
    //TODO: THIS
    return 0;
  }

  public void extendArm(DoubleSupplier powerSupplier){
    // m_actuator.set(powerSupplier.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("encoderRpos", m_rightEncoder.getPosition() + " ticks(?)");
  }
}
