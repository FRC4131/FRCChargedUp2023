// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPosition;

public class ArmSubsystem extends SubsystemBase {

  // 5:1, 4:1, 3:1 (gearboxes) + 3.86:1 (15-tooth to 58-tooth output gears) gear
  // ratios ratio
  private final double ARM_MOTOR_GEAR_RATIO = 5.0 / 1.0 * 4.0 / 1.0 * 3.0 / 1.0 * 58.0 / 15.0;

  // Follower motor
  private CANSparkMax m_leftRot = new CANSparkMax(59, MotorType.kBrushless);

  // Leader motor
  private CANSparkMax m_rightRot = new CANSparkMax(58, MotorType.kBrushless);

  private SparkMaxPIDController m_rightRotPIDController;

  private RelativeEncoder m_rightEncoder;

  private RelativeEncoder m_leftEncoder;

  // private DigitalInput m_minLimitSwitch = new DigitalInput(0);//TODO: Channel #s
  // private DigitalInput m_maxLimitSwitch = new DigitalInput(1);//TODO: Channel #s
  

  private boolean bool;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    SmartDashboard.putNumber("CLAMP ARM SPEED", 1);
    // m_actuator.configFeed
    m_rightEncoder = m_rightRot.getEncoder();
    m_leftEncoder = m_leftRot.getEncoder();
    m_rightRotPIDController = m_leftRot.getPIDController();
    m_rightEncoder.setPositionConversionFactor(1);
    m_rightEncoder.setVelocityConversionFactor(1);
    m_leftEncoder.setPositionConversionFactor(1);
    m_leftEncoder.setVelocityConversionFactor(1);
    m_rightRotPIDController.setOutputRange(-1, 1);
    m_rightRotPIDController.setSmartMotionMaxAccel(7500, 0);
    m_rightRotPIDController.setSmartMotionMaxVelocity(5000, 0);
    m_rightRotPIDController.setSmartMotionMinOutputVelocity(0, 0);
    m_rightRotPIDController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);

    // // Smart Motion
    // m_rightRotPIDController.setP(4e-9);
    // m_rightRotPIDController.setI(0);
    // m_rightRotPIDController.setD(0);
    // m_rightRotPIDController.setFF(2e-4);

    // Position
    m_rightRotPIDController.setP(5e-2);
    m_rightRotPIDController.setI(0);
    m_rightRotPIDController.setD(0);
    m_rightRotPIDController.setFF(0);

    SmartDashboard.putBoolean("bool", bool);
    // m_leftRot.setIdleMode(IdleMode.kCoast);

    // m_leftRot.fp
    resetPosition(47.5);

    m_rightRot.setIdleMode(IdleMode.kBrake);
    m_leftRot.setIdleMode(IdleMode.kBrake);
    m_rightRot.enableSoftLimit(SoftLimitDirection.kForward, false);
    m_rightRot.enableSoftLimit(SoftLimitDirection.kReverse, false);
    m_rightRot.setSoftLimit(SoftLimitDirection.kForward, (float)angleToMotorRotations(ArmPosition.MAX.rotation - .1));
    m_rightRot.setSoftLimit(SoftLimitDirection.kReverse, (float)angleToMotorRotations(ArmPosition.MIN.rotation + .1));

    // m_actuatorPIDController = m_actuator.getPIDController();

    // m_leftRot.follow(m_rightRot, true);
    // m_leftRot.follow(m_leftRot);
    // m_rightRot.follow(m_leftRot, true);

    m_rightRot.burnFlash();
    m_leftRot.burnFlash();
    m_rightRotPIDController.setReference(0, ControlType.kDutyCycle);
  }

  public void adjustSpeed(double maxVelocity, double maxAcceleration){
    m_rightRotPIDController.setSmartMotionMaxVelocity(maxVelocity, 0);
    m_rightRotPIDController.setSmartMotionMaxAccel(maxAcceleration, 0);

    SmartDashboard.putNumber("ARM MAX VEL LIMIT", m_rightRotPIDController.getSmartMotionMaxVelocity(0));
  }

  public void clampSpeed(double outputPercent){
    m_rightRotPIDController.setOutputRange(-outputPercent, outputPercent);
  }

  public void rotateArm(double power) {
    // m_rightRot.set(powerSupplier.getAsDouble() * 0.1);
    m_rightRotPIDController.setReference(power, ControlType.kDutyCycle);
    SmartDashboard.putNumber("TOSE", power);
  }

  public void snapToAngle(double angleDegrees) {
    double encoderRotations = angleToMotorRotations(angleDegrees);
    m_rightRotPIDController.setReference(encoderRotations, ControlType.kPosition);
  };

  public void resetPosition(double position) {
    m_rightEncoder.setPosition(angleToMotorRotations(position));
    m_leftEncoder.setPosition(angleToMotorRotations(position));
    // m_rightRotPIDController.setReference(0, ControlType.kPosition);
  }

  private double angleToMotorRotations(double angleDegrees) {
    return angleDegrees * ARM_MOTOR_GEAR_RATIO / 360.0;
  }

  /**
   * 
   * @param angle to rotate to in degrees
   * @return An instant command
   */
  public CommandBase rotateCommand(double angle) {
    return runOnce(() -> {
      snapToAngle(angle);
    });
  }

  public double getArmAngle(){
    return m_leftEncoder.getPosition() / ARM_MOTOR_GEAR_RATIO * 360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if(m_maxLimitSwitch.get()){
    //   m_rightEncoder.setPosition(angleToMotorRotations(ArmPosition.MAX.rotation));
    // }
    // if(m_minLimitSwitch.get()){
    //   m_rightEncoder.setPosition(angleToMotorRotations(ArmPosition.MIN.rotation));
    // }
      
      // clampSpeed(SmartDashboard.getNumber("CLAMP ARM SPEED", 1));


    SmartDashboard.putNumber("Arm Velocity", m_rightEncoder.getVelocity());
    SmartDashboard.putString("Arm Position",
        m_rightEncoder.getPosition() / ARM_MOTOR_GEAR_RATIO * 360 + " Degrees");
    SmartDashboard.putString("Left Arm Motor Pos", m_leftEncoder.getPosition() / ARM_MOTOR_GEAR_RATIO * 360 + "Degrees");
  }

  public CommandBase resetEncoder(double position) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          resetPosition(position);
        });
  }
}
