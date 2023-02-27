// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax m_wristController = new CANSparkMax(57, MotorType.kBrushless);
  private RelativeEncoder m_Encoder;
  private SparkMaxPIDController m_WristPID;
  //clockwise channel needs to be changed
  private DigitalInput clockwiseLimit = new DigitalInput(2);
  private DigitalInput counterClockwiseLimit = new DigitalInput(1);
  private final double WRIST_MOTOR_GEAR_RATIO = 25;
  private boolean isMovingClockwise;

  public WristSubsystem() {
    m_Encoder = m_wristController.getEncoder();
    m_WristPID = m_wristController.getPIDController();

    m_Encoder.setVelocityConversionFactor(0.00818123109638691);
    m_WristPID.setOutputRange(-1, 1);
    m_WristPID.setSmartMotionMaxAccel(25, 0);
    m_WristPID.setSmartMotionMaxVelocity(30, 0);
    m_WristPID.setSmartMotionMinOutputVelocity(0, 0);
    m_WristPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_WristPID.setP(0.0009);
    m_WristPID.setI(0);
    m_WristPID.setD(0);
    m_WristPID.setFF(0.011);

    m_wristController.setInverted(false);
    m_wristController.burnFlash();
  }

  public void wristSpeed(double d) {
    m_WristPID.setReference(d, ControlType.kDutyCycle);
  }

  public void resetPosition(){
    m_Encoder.setPosition(0);
  }

  /**
   * Sets the setPoint for the PID controller to run to the specified position
   * @param desiredAngle Angle to rotate to in degrees
   */
  public void rotateTo(double desiredAngle){
    m_WristPID.setReference(angleToMotorRotations(desiredAngle), ControlType.kSmartMotion);
  }


  public void rotate(){ //negative clockwise, positive counterclockwise
    isMovingClockwise = getCounterClockwiseSwitch();
    if (getClockwiseSwitch() && !isMovingClockwise)
    {
      m_WristPID.setReference(-8, ControlType.kSmartVelocity);
    }
    else if (getCounterClockwiseSwitch() && isMovingClockwise)
    {
      m_WristPID.setReference(8, ControlType.kSmartVelocity);
    }
    else if (!(getClockwiseSwitch() || getClockwiseSwitch())){
      m_WristPID.setReference(8, ControlType.kSmartVelocity);
    }

  }

  public void stopRotate(){
    m_WristPID.setReference(-8, ControlType.kSmartVelocity);
  }

  public void rotateClockwise(){
    m_WristPID.setReference(-8, ControlType.kSmartVelocity);
  }

  public void rotateCounterClockwise(){
    m_WristPID.setReference(8, ControlType.kSmartVelocity);
  }


  public void checkLimitSwitch(){
    if (getClockwiseSwitch() && isMovingClockwise)
    {
      m_WristPID.setReference(0, ControlType.kSmartVelocity);
    }
    else if (getCounterClockwiseSwitch() && !isMovingClockwise)
    {
      m_WristPID.setReference(0, ControlType.kSmartVelocity);
    }
  }



  public boolean getClockwiseSwitch(){
    return !clockwiseLimit.get();
  }

  public boolean getCounterClockwiseSwitch(){
    return !counterClockwiseLimit.get();
  }


  private double angleToMotorRotations(double angleDegrees) {
    return angleDegrees * WRIST_MOTOR_GEAR_RATIO / 360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", angleToMotorRotations(m_Encoder.getPosition()));
    SmartDashboard.putNumber("Wrist Velocity", m_Encoder.getVelocity());
    SmartDashboard.putBoolean("Clockwise Switch", getClockwiseSwitch());
    SmartDashboard.putBoolean("Counterclockwise Switch", getCounterClockwiseSwitch());
    checkLimitSwitch();

  }
}
