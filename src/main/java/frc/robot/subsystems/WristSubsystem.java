// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax m_wristController = new CANSparkMax(57, MotorType.kBrushless);

  public WristSubsystem() {}

  public void wristSpeed(double d) {
    m_wristController.set(d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}