// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {
  
  private TalonSRX m_actuator = new TalonSRX(30);
  private SparkMaxPIDController m_actuatorPIDController;

  /** Creates a new ExtensionSubsystem. */
  public ExtensionSubsystem() {}
  
  
  public void extendTo(double length) {
    m_actuatorPIDController.setReference(lengthToUnits(length), ControlType.kPosition);
  }

  public void extendArm(double power) {
    m_actuator.set(TalonSRXControlMode.PercentOutput, power);
  }
  
  private double lengthToUnits(double length) {
    // TODO: THIS
    return 0;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
