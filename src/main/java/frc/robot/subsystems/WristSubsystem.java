// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  CANSparkMax m_wristController = new CANSparkMax(60, MotorType.kBrushless);
  private RelativeEncoder m_Encoder;
  private SparkMaxPIDController m_WristPID;
  // clockwise channel needs to be changed
  private DigitalInput clockwiseLimit = new DigitalInput(2);
  private DigitalInput counterClockwiseLimit = new DigitalInput(1);
  private final double WRIST_MOTOR_GEAR_RATIO = 25;
  private boolean isMovingClockwise = false;

  public WristSubsystem() {
    m_Encoder = m_wristController.getEncoder();
    m_WristPID = m_wristController.getPIDController();

    m_Encoder.setVelocityConversionFactor(0.00818123109638691);
    m_WristPID.setOutputRange(-1, 1);
    m_WristPID.setSmartMotionMaxAccel(25, 0);
    m_WristPID.setSmartMotionMaxVelocity(20, 0);
    m_WristPID.setSmartMotionMinOutputVelocity(0, 0);
    m_WristPID.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

    m_WristPID.setP(0.00005);
    m_WristPID.setI(0);
    m_WristPID.setD(0);
    m_WristPID.setFF(0.011);

    m_WristPID.setSmartMotionAllowedClosedLoopError(0.01, 0);

    m_wristController.setInverted(false);
    m_wristController.burnFlash();

    resetPosition(0);
  }

  public void wristSpeed(double d) {
    m_WristPID.setReference(d, ControlType.kDutyCycle);
  }

  public void resetPosition(double angleDegrees) {
    m_Encoder.setPosition(angleToMotorRotations(angleDegrees));
  }

  /**
   * Sets the setPoint for the PID controller to run to the specified position
   * 
   * @param desiredAngle Angle to rotate to in degrees
   */
  public void rotateTo(double desiredAngle) {
    m_WristPID.setReference(angleToMotorRotations(desiredAngle), ControlType.kSmartMotion);
  }

  public void rotate() { // negative clockwise, positive counterclockwise
    isMovingClockwise = getCounterClockwiseSwitch();
    if (getClockwiseSwitch() && !isMovingClockwise) {
      m_WristPID.setReference(-15, ControlType.kSmartVelocity);
    } else if (getCounterClockwiseSwitch() && isMovingClockwise) {
      m_WristPID.setReference(15, ControlType.kSmartVelocity);
    } else if (!(getClockwiseSwitch() || getClockwiseSwitch())) {
      m_WristPID.setReference(15, ControlType.kSmartVelocity);
    }

  }

  public void stopRotate() {
    SmartDashboard.putBoolean("Wrist Give it a Wrist", true);
    isMovingClockwise = isMovingClockwise ? false : true;
    m_WristPID.setReference(0, ControlType.kDutyCycle);
  }

  public void rotateClockwise() {
    isMovingClockwise = false;
    m_WristPID.setReference(-15, ControlType.kSmartVelocity);
  }

  public void rotateCounterClockwise() {
    isMovingClockwise = true;
    m_WristPID.setReference(15, ControlType.kSmartVelocity);
  }

  public void checkLimitSwitch() {
    if (getClockwiseSwitch() && isMovingClockwise) {
      stopRotate();

    } else if (getCounterClockwiseSwitch() && !isMovingClockwise) {
      stopRotate();
    }
    if ((getClockwiseSwitch() || getCounterClockwiseSwitch()) && m_wristController.getOutputCurrent() > 25) {
      stopRotate();
    }
  }

  public boolean getClockwiseSwitch() {
    return !clockwiseLimit.get();
  }

  public boolean getCounterClockwiseSwitch() {
    return !counterClockwiseLimit.get();
  }

  private double angleToMotorRotations(double angleDegrees) {
    return angleDegrees * WRIST_MOTOR_GEAR_RATIO / 360.0 / 22.5;
  }

  public void setReference(double angleDegrees) {
    m_WristPID.setReference(angleToMotorRotations(angleDegrees), ControlType.kSmartMotion);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Angle", m_Encoder.getPosition() / WRIST_MOTOR_GEAR_RATIO * 360 * 22.5);
    SmartDashboard.putBoolean("Clockwise Switch", getClockwiseSwitch());
    SmartDashboard.putBoolean("Counterclockwise Switch", getCounterClockwiseSwitch());
    SmartDashboard.putNumber("Wrist Current", m_wristController.getOutputCurrent());
    SmartDashboard.putNumber("Victor tose", angleToMotorRotations(-90));
    SmartDashboard.putNumber("Wrist Real Position", m_Encoder.getPosition());
    checkLimitSwitch();

  }
}
