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

  private static final int GEAR_RATIO = 125;

  CANSparkMax m_clawController = new CANSparkMax(60, MotorType.kBrushless);
  RelativeEncoder m_clawEncoder;
  SparkMaxPIDController m_clawPID;

  DigitalInput m_maxLimit = new DigitalInput(0);
  DigitalInput m_minLimit = new DigitalInput(0);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem(){

    m_clawEncoder = m_clawController.getEncoder();
    m_clawPID = m_clawController.getPIDController();

    m_clawController.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_clawController.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_clawController.setSoftLimit(SoftLimitDirection.kForward, (float)angleToMotorRotations(180 - .1));
    m_clawController.setSoftLimit(SoftLimitDirection.kReverse, (float)angleToMotorRotations(.1));

    m_clawPID.setP(1);
  }

  public void intakeSpeed(double d) {
    m_clawController.set(d);
  }


  public double angleToMotorRotations(double angle){
    return angle / 360 / GEAR_RATIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(m_maxLimit.get()){
      m_clawEncoder.setPosition(angleToMotorRotations(180));
    }
    if(m_minLimit.get()){
      m_clawEncoder.setPosition(angleToMotorRotations(0));
    }
  }
}
