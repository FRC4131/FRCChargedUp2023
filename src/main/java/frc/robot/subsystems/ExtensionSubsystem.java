// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class ExtensionSubsystem extends SubsystemBase {

  private TalonSRX m_actuator = new TalonSRX(30);
  
  private DigitalInput m_forwardLimit = new DigitalInput(0); //arbitrary channel values for now
  private DigitalInput m_reverseLimit = new DigitalInput(1);
  
  private double desired;
  private PIDController m_actuatorPIDController = new PIDController(
      1,
      0,
      0);

  private final double GEAR_RATIO = 5 * 7;

  /** Creates a new ExtensionSubsystem. */
  public ExtensionSubsystem() {
    desired = 0;
    m_actuator.configSelectedFeedbackCoefficient(1);
    // m_actuator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);
    // m_actuator.configSelectedFeedbackCoefficient(100 / (1024 * GEAR_RATIO *
    // 0.748), 1, 0);

    m_actuator.setSensorPhase(true);
    m_actuator.setInverted(false);

    m_actuator.configNeutralDeadband(0.001);

    m_actuator.configNominalOutputForward(0);
    m_actuator.configNominalOutputReverse(0);
    m_actuator.configPeakOutputForward(1);
    m_actuator.configPeakOutputReverse(-1);

    m_actuator.selectProfileSlot(1, 0);
    m_actuator.config_kP(1, 0.0165);
    m_actuator.config_kI(1, 0);
    m_actuator.config_kD(1, 0);
    m_actuator.config_kF(1, 0);

    m_actuator.configMotionCruiseVelocity(100000);
    m_actuator.configMotionAcceleration(200000);

   m_actuator.configForwardSoftLimitEnable(true);
   m_actuator.configForwardSoftLimitThreshold(lengthToUnits(ArmPosition.MAX.length - 0.1));
   m_actuator.configReverseSoftLimitEnable(true);
   m_actuator.configReverseSoftLimitThreshold(lengthToUnits(.1));
  }

  /**
   * 
   * @param desired position of the telescope in inches from retracted
   */
  public void extendTo(double desired) {
    m_actuator.set(ControlMode.MotionMagic, -desired * (1024 * GEAR_RATIO * 0.748));
    this.desired = desired;
  }

  /** stop trying if encoder is within 700 units of goal */
  public boolean atGoal() {
    return Math.abs((-lengthToUnits(desired)) - m_actuator.getSelectedSensorPosition()) < 700;
  }

  public void extendArm(double power)
  {
   m_actuator.set(TalonSRXControlMode.PercentOutput, power);
  }


  private double lengthToUnits(double inches) {
    // TODO: THIS
    return inches * 1024 * GEAR_RATIO * .748;
  }

  /**
   * @return The position of the motor (in 1024 encoder ticks per rev)
   */
  public double getPosition() {
    return -m_actuator.getSelectedSensorPosition(0) * (100 / (1024 * GEAR_RATIO * 0.748));
  }

  public void resetEncoder() {
    m_actuator.setSelectedSensorPosition(0);
  }

 /* public boolean getForwardOutput()
  {
    return m_actuator.getSensorCollection().isFwdLimitSwitchClosed();
  }

  public boolean getReverseOutput()
  {
    return m_actuator.getSensorCollection().isRevLimitSwitchClosed();
  } */

  /**
   * 
   * @param length to extend to in inches
   * @return An instant command
   */
  public CommandBase extendCommand(double length) {
    return runOnce(() -> {
      extendTo(length);
    });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // m_actuator.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
   // m_actuator.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
   // SmartDashboard.putBoolean("ForwardSwitch", getForwardOutput());
   // SmartDashboard.putBoolean("ReverseSwitch", getReverseOutput());

    if (m_forwardLimit.get())
    {
      m_actuator.setSelectedSensorPosition(lengthToUnits(ArmPosition.MAX.length));
    }
    if (m_reverseLimit.get())
    {
      m_actuator.setSelectedSensorPosition(lengthToUnits(ArmPosition.MIN.length));
    }
    m_actuator.set(TalonSRXControlMode.PercentOutput, 1/GEAR_RATIO);

    SmartDashboard.putNumber("Telescope Position", getPosition());
    SmartDashboard.putBoolean("Telescope AtGoal", atGoal());
    SmartDashboard.putNumber("OFFSET BRUH",
        Math.abs((-desired * (1024 * GEAR_RATIO * 0.748)) - m_actuator.getSelectedSensorPosition()));
  }
}
