// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax m_wristController = new CANSparkMax(57, MotorType.kBrushless);
  //private SparkMaxLimitSwitch m_forwardLimit = m_wristController.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  //private SparkMaxLimitSwitch m_reverseLimit = m_wristController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  private DigitalInput m_forwardLimit = new DigitalInput(0); //arbitrary channels for now
  private DigitalInput m_reverseLimit = new DigitalInput(1);
  public WristSubsystem() {}

  public void wristSpeed(double d) 
  {
    if (d > 0)
    {
      if (m_forwardLimit.get())
      {
        m_wristController.set(0);
      }
      else 
      {
        m_wristController.set(d);
      }
    }
      else
      {
        if (m_reverseLimit.get())
        {
          m_wristController.set(0);
        }
        else 
        {
          m_wristController.set(d);
        }
      } 
    }
  
  

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    //SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
  
    
  }
}
