// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {


  CANSparkMax m_clawController = new CANSparkMax(60, MotorType.kBrushless);
  RelativeEncoder m_clawEncoder;
  SparkMaxPIDController m_clawPID;

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem(){

    m_clawEncoder = m_clawController.getEncoder();
    m_clawPID = m_clawController.getPIDController();
    m_clawController.setSmartCurrentLimit(30, 40);

    m_clawController.burnFlash();


    m_clawPID.setP(1);
  }

  public void intakeSpeed(double d) {
    m_clawController.set(d * 1);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
