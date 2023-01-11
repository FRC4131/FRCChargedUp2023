package frc.lib.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import frc.lib.util.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.Constants;
import frc.robot.Robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import static frc.robot.Constants.Swerve.*;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX jacobBublitz;
    private CANSparkMax m_angleMotor;
    private CANSparkMax m_driveMotor;
    private RelativeEncoder angleEncoder;
    private RelativeEncoder driveEncoder;
    private PIDController turningPidController;
    private SparkMaxPIDController builtinTurningPidController;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        /* Angle Motor Config */
        m_angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        m_driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        configDriveMotor();

        // Create PID controller on ROBO RIO
        turningPidController = new PIDController(0.5, 0, 0);

        // Tell PID controller that it is a *wheel*
        turningPidController.enableContinuousInput(0, 2 * Math.PI);

        // -----SPARK-MAX-PID-----//

        builtinTurningPidController = m_angleMotor.getPIDController();

        // Set PID values for the simulated Spark max PID
        builtinTurningPidController.setP(0.5);
        builtinTurningPidController.setI(0.0);
        builtinTurningPidController.setD(0.005);
        builtinTurningPidController.setIZone(0.0);
        builtinTurningPidController.setFF(0.0);
        builtinTurningPidController.setOutputRange(-1, 1);

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        /*
         * This is a custom optimize function, since default WPILib optimize assumes
         * continuous controller which CTRE and Rev onboard is not
         */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
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

    public double getTurningPosition() {
          return angleEncoder.getPosition();
        }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return angleEncoder.getVelocity();
    }

    private void configAngleMotor() {
        m_angleMotor.setInverted(angleMotorInvert);

        // Change conversion factors for neo turning encoder - should be in radians!
        angleEncoder.setPositionConversionFactor(kTurningEncoderRot2Rad);
        angleEncoder.setVelocityConversionFactor(kTurningEncoderRPM2RadPerSec);
    }

    private void configDriveMotor() {
        m_driveMotor.setInverted(driveMotorInvert);
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
}