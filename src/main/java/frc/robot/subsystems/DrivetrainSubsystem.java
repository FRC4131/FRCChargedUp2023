package frc.robot.subsystems;

import frc.lib.util.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public AHRS m_navX;
    public frc.lib.util.SwerveModule[] m_SwerveMods;

    public DrivetrainSubsystem() {
        
        m_SwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        for (SwerveModule mod : m_SwerveMods){
            mod.reset();
        }
    }

    /**
     * 
     * @param translation X and Y values (multiply by max speed)
     * @param rotation  Theta value (multiply by max angular velocity)
     * @param currentRotation The current rotation/theta of the robot (i.e. the yaw from the gyro)
     * @param fieldRelative Is field relative
     * @param isOpenLoop 
     */
    public void drive(Translation2d translation, double rotation, Rotation2d currentRotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        currentRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates);
    }

    public void drive(ChassisSpeeds chassisSpeeds)
    {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            chassisSpeeds 
        );
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_SwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_SwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic() {
        for (SwerveModule mod : m_SwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Position", mod.getDrivePosition());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Turn Position", mod.getTurningPosition());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }
        
    }
}