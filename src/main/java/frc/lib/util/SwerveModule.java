package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.util.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX jacobBublitz;
    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private RelativeEncoder driveEncoder;
    private CANCoder angleEncoder;
    private ProfiledPIDController turningPidController;

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Motor & Encoder Config */
        m_angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        m_driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        driveEncoder = m_driveMotor.getEncoder(com.revrobotics.SparkMaxRelativeEncoder.Type.kHallSensor, 42);
        angleEncoder = new CANCoder(moduleConstants.cancoderID);

        configAngleMotor();
        configDriveMotor();

        // Create PID controller on ROBO RIO
        turningPidController = new ProfiledPIDController(0.4, 0, 0,
                new TrapezoidProfile.Constraints(20 * 2 * Math.PI, 20 * 2 * Math.PI));

        // Tell PID controller that it is a *wheel*
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        lastAngle = getState().angle;

        angleEncoder.configMagnetOffset(moduleConstants.angleOffset.getDegrees());
        driveEncoder.setPosition(0);
        angleEncoder.setPosition(0);
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        desiredState = CTREModuleState.optimize(desiredState, Rotation2d.fromRadians(getTurningPosition()));
        setAngle(desiredState);
        setSpeed(desiredState);
    }

    private void setSpeed(SwerveModuleState desiredState) {
        double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
        m_driveMotor.set(percentOutput);
    }

    private void setAngle(SwerveModuleState desiredState) {
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
                ? lastAngle
                : desiredState.angle; // Prevent rotating module if speed is less then 1%. Prevents Jittering.

        m_angleMotor.set(turningPidController.calculate(getTurningPosition(),
                desiredState.angle.getRadians()));
        lastAngle = angle;
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public void rawSet(double drive, double turn) {
        m_driveMotor.set(drive);
        m_angleMotor.set(turn);
    }

    public double getTurningPosition() {
        return angleEncoder.getAbsolutePosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return angleEncoder.getVelocity();
    }

    private void configAngleMotor() {
        m_angleMotor.setInverted(angleMotorInvert);
    }

    private void configDriveMotor() {
        m_driveMotor.setInverted(driveMotorInvert);
        m_driveMotor.setIdleMode(IdleMode.kCoast);
        m_driveMotor.burnFlash();
        driveEncoder.setPositionConversionFactor(kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(kDriveEncoderRPM2MeterPerSec);

        // Change conversion factors for neo turning encoder - should be in radians!
        driveEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad);
        driveEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public void stop() {
        m_angleMotor.stopMotor();
        m_driveMotor.stopMotor();
    }
}