// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final int kMacropadPort = 2;
    }

    public static final class VisionConstants {
        public static final double GOAL_RANGE_METERS = 5;
    }

    /**DEGREES AND INCHES!!
     * <p>
     * TEMPORARY MAYBE
    */
    public enum ArmPosition {
        LOW(110,1.0),
        MEDIUM(-61, 1.5),
        HIGH(-53, 19.65),
        STOW(110,0),
        ZEROES(0,0),
        MIN(-30, 0),
        MAX(30, 18),
        DOUBLESUB(-58, 0),
        FLOOR(110,15), //guestimate 
        DEFAULT(0,0), //guestimate 
        CUBENODEMEDIUM(-67, 1.5),
        CUBENODEHIGH(-59, 18.8),
        MEDIUMCOMMIT(-70, 1.5),
        HIGHCOMMIT(-64, 19.65),
        SHOOTPOSITION(-70, 0),
        INTAKEBACK(-114.5,0), 
        INTAKEFRONT(117, 5.7),
        SALUTE(90,0),
        AUTONCUBEHIGH(-58, 10);


        public final double rotation;
        public final double length;

        ArmPosition(double rot, double ext) {
            rotation = rot;
            length = ext;
        }
    }

    public enum GridPositions{
        RED1(15.1, 1.08),
        RED2(15.1, 2.75),
        RED3(15.1, 4.45),
        BLUE6(1.92, 4.45),
        BLUE7(1.92, 2.75),
        BLUE8(1.92, 1.08),
        REDDUBSUB(1.36195, 7.4),
        BLUEDUBSUB(15.178784, 7.4);
        

        public final double x;
        public final double y;

        GridPositions(double x, double y){
                this.x = x;
                this.y = y;
        }
    }

    public static final class ArmConstants{
        public static final double DEFAULT_MAX_VELOCITY = 4500;
        public static final double DEFAULT_MAX_ACCEL = 4500;
        public static final double FAST_MAX_VELOCITY = 5000;
        public static final double FAST_MAX_ACCEL = 7500;
    }

    public static final class AprilTagConstants {
        public static final AprilTag tag1 = new AprilTag(1,
                new Pose3d(15.513558, 1.071626, 0.462788,
                        new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));

        public static final AprilTag tag2 = new AprilTag(2,
                new Pose3d(15.513558, 2.748026, 0.462788,
                        new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));

        public static final AprilTag tag3 = new AprilTag(3,
                new Pose3d(15.513558, 4.424426, 0.462788,
                        new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));

        public static final AprilTag tag4 = new AprilTag(4,
                new Pose3d(16.178784, 6.749796, 0.695452,
                        new Rotation3d(new Quaternion(0.0, 0.0, 0.0, 1.0))));

        public static final AprilTag tag5 = new AprilTag(5,
                new Pose3d(0.36195, 6.749796, 0.695452,
                        new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

        public static final AprilTag tag6 = new AprilTag(6,
                new Pose3d(1.02743, 4.424426, 0.462788,
                        new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

        public static final AprilTag tag7 = new AprilTag(7,
                new Pose3d(1.02743, 2.748026, 0.462788,
                        new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));

        public static final AprilTag tag8 = new AprilTag(8,
                new Pose3d(1.02743, 1.071626, 0.462788,
                        new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))));
    }

    public static final class Swerve {
        // 1.15
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.85714286; // old value 1 / 5.8462
        public static final double kTurningMotorGearRatio = 1 / 12.8; // old value 1 / 18.0
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule = // TODO: This must be tuned to specific robot
                COTSFalconSwerveConstants.SDSMK3(COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Fast);

        /* Drivetrain Constants IN METERS */
        public static final double trackWidth = Units.inchesToMeters(21); // TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(24);; // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 120.0 *
                COTSFalconSwerveConstants.driveGearRatios.SDSMK3_Fast *
                wheelCircumference;
        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 4.4 /
                Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         */
        public static final double driveKS = (0.32 / 12); // TODO: This must be tuned to specific robot
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.4; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 19;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((-4.85 * Math.PI) / 4);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 20;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 15;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians(1.13 * Math.PI);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Left Module - Module 2 */
        public static final class Mod2 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((-3.45 * Math.PI) / 4);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { // TODO: This must be tuned to specific robot
            public static final int driveMotorID = 14;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromRadians((0.75 * Math.PI) / 2);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
        }
    }
}