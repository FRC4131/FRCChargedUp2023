// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
//import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */

  private static final int GEAR_RATIO = 125;
  private CANSparkMax m_wristController = new CANSparkMax(57, MotorType.kBrushless);
  //private SparkMaxLimitSwitch m_forwardLimit = m_wristController.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
  //private SparkMaxLimitSwitch m_reverseLimit = m_wristController.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

  private RelativeEncoder m_wristEncoder;

  DigitalInput m_maxLimit = new DigitalInput(0);
  DigitalInput m_minLimit = new DigitalInput(0);
  public WristSubsystem() {

    m_wristEncoder = m_wristController.getEncoder();

    m_wristController.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_wristController.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_wristController.setSoftLimit(SoftLimitDirection.kForward, (float)angleToMotorRotations(180 - .1));
    m_wristController.setSoftLimit(SoftLimitDirection.kReverse, (float)angleToMotorRotations(.1));
  }


  public double angleToMotorRotations(double angle){
    return angle / 360 / GEAR_RATIO;
  }

  public void wristSpeed(double d) 
  {
    /*if ((!m_forwardLimit.get()) && (!m_reverseLimit.get())){
       wristSpeed(d);
    } else if (m_forwardLimit.get()) {
        if (d < 0)
        {
          wristSpeed(d);
        }
        else{
          wristSpeed(0);
        }    
    } else if (m_reverseLimit.get()){
        if (d > 0)
        {
          wristSpeed(d);
        }
        else {
          wristSpeed(0);
        }
    }

    if(m_forwardLimit.get() && d > 0)
    {
      d = 0;
    }
    if (m_reverseLimit.get() && d < 0)
    {
      d = 0;
    }

    wristSpeed(d);

    */
  }

  //WARNING: THIS JUST SETS THE POWER IN THE DIRECTION TOWARDS DESIRED ANGLE
  //THIS WILL ONLY END DUE TO LIMIT SWITCH BEING HIT, AT 0 or 180 degrees
  public void wristToAngle(double angle){
    m_wristController.set((angleToMotorRotations(angle)-m_wristEncoder.getPosition())/180);
  }

  /*public DigitalInput getForwardLimit()
  {
    return m_forwardLimit;
  }

  public DigitalInput getReverseLimit()
  {
    return m_reverseLimit;
  } */

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    //SmartDashboard.putBoolean("Forward Limit Switch", m_forwardLimit.isPressed());
    //SmartDashboard.putBoolean("Reverse Limit Switch", m_reverseLimit.isPressed());
      
      
    if(m_maxLimit.get()){
      m_wristEncoder.setPosition(angleToMotorRotations(180));
    }
    if(m_minLimit.get()){
      m_wristEncoder.setPosition(angleToMotorRotations(0));
    }
    
  }
}
