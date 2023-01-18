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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public AHRS m_navX;
    public frc.lib.util.SwerveModule[] mSwerveMods;

    public DrivetrainSubsystem() {
        
        mSwerveMods = new SwerveModule[] {
                new SwerveModule(
                    0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
        for (SwerveModule mod : mSwerveMods){
            mod.reset();
        }
    }

    /**
     * 
     * @param translation X and Y values (multiply by max speed)
     * @param rotation  Theta value (multiply by max angular velocity)
     * @param fieldRelative Is field relative
     * @param isOpenLoop 
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        getYaw())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }


    

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

   

    @Override
    public void periodic() {

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Position", mod.getDrivePosition());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Turn Position", mod.getTurningPosition());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putNumber("ROT VALUE", m_navX.getYaw());
        SmartDashboard.putNumber("x", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odom Rotation", swerveOdometry.getPoseMeters().getRotation().getDegrees());
    }
}