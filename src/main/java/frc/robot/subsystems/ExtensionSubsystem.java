// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtensionSubsystem extends SubsystemBase {

  private TalonSRX m_actuator = new TalonSRX(30);
  private double desired;
  private DigitalInput maxLimit = new DigitalInput(0);
  private DigitalInput minLimit = new DigitalInput(1);// TODO: Channel #s

  private double GEAR_RATIO = 5 / 1 * 7 / 1;

  /** Creates a new ExtensionSubsystem. */
  public ExtensionSubsystem() {
    desired = 0;
    m_actuator.configSelectedFeedbackCoefficient(1);
    // m_actuator.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 1000);
    // m_actuator.configSelectedFeedbackCoefficient(100 / (1024 * GEAR_RATIO *
    // 0.748), 1, 0);

    m_actuator.setSensorPhase(false);
    m_actuator.setInverted(true);

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

    m_actuator.configPeakCurrentLimit(40);
    m_actuator.configPeakCurrentDuration(250);
    m_actuator.configContinuousCurrentLimit(0);
    m_actuator.enableCurrentLimit(true);

  }

  /**
   * 
   * @param desired position of the telescope in inches from retracted
   */
  public void extendTo(double desired) {
    m_actuator.set(ControlMode.MotionMagic, -desired * (1024 * GEAR_RATIO * 0.748));
    this.desired = desired;
  }

  public boolean atGoal() {
    return Math.abs((-desired * (1024 * GEAR_RATIO * 0.748)) - m_actuator.getSelectedSensorPosition()) < 700;
  }

  public void extendArm(double power) {
    m_actuator.set(TalonSRXControlMode.PercentOutput, power);
  }

  private double lengthToUnits(double length) {
    // TODO: THIS
    return length / 1024 / GEAR_RATIO;
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
    SmartDashboard.putNumber("Telescope Position", getPosition());
    SmartDashboard.putNumber("Telescope Current", m_actuator.getSupplyCurrent());
    SmartDashboard.putNumber("Telescope Temp", m_actuator.getTemperature());

    if(maxLimit.get())
      m_actuator.setSelectedSensorPosition(-21.8 * (1024 * GEAR_RATIO * 0.748));

    if(minLimit.get())
      m_actuator.setSelectedSensorPosition(0);
  }
}
