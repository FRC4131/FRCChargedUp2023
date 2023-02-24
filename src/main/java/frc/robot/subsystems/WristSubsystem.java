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
  private DigitalInput clockwiseLimit = new DigitalInput(4);
  private DigitalInput counterClockwiseLimit = new DigitalInput(5);// TODO: Channel #s
  private final double WRIST_MOTOR_GEAR_RATIO = 125;

  public WristSubsystem() {
    m_Encoder = m_wristController.getEncoder();
    m_WristPID = m_wristController.getPIDController();

    m_WristPID.setOutputRange(-1, 1);
    m_WristPID.setSmartMotionMaxAccel(7500, 0);
    m_WristPID.setSmartMotionMaxVelocity(10000, 0);
    m_WristPID.setSmartMotionMinOutputVelocity(0, 0);
    m_WristPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_WristPID.setP(4e-9);
    m_WristPID.setI(0);
    m_WristPID.setD(0);
    m_WristPID.setFF(9.9e-5);

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

  private double angleToMotorRotations(double angleDegrees) {
    return angleDegrees * WRIST_MOTOR_GEAR_RATIO / 360.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", angleToMotorRotations(m_Encoder.getPosition()));
  }
}
