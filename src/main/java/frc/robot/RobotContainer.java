// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.CommandMacroPad;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CalibrateOdometryCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.PPCommand;
import frc.robot.commands.SeekingCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_drivetrainSubsystem,
      m_visionSubsystem);

  private SendableChooser<Command> m_autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  private final CommandMacroPad m_macroPad = new CommandMacroPad(1);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    DoubleSupplier testLeftY = () -> m_driverController.getLeftY() * MAX_VELOCITY_METERS_PER_SECOND;
    DoubleSupplier testLeftX = () -> m_driverController.getLeftX() * MAX_VELOCITY_METERS_PER_SECOND;
    DoubleSupplier testRightX = () -> m_driverController.getRightX() * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

    ShuffleboardTab tab = Shuffleboard.getTab("tab");
    tab.addNumber("testLeftY", testLeftY);
    tab.addNumber("testLeftX", testLeftX);
    tab.addNumber("testRightX", testRightX);

    addAuton();
    SmartDashboard.putData(m_autoChooser);
    // Configure the trigger bindings
    configureBindings();
    setDefaultCommands();
  }

  public void setDefaultCommands() {
    double speedCap = Constants.Swerve.maxSpeed;
    m_drivetrainSubsystem
        .setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY(), false) * speedCap,
            () -> -modifyAxis(m_driverController.getLeftX(), false) * speedCap,
            () -> -modifyAxis(m_driverController.getRightX(), false) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> m_driverController.getLeftTriggerAxis(),
            true));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void addAuton() {
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("PathplannerAuton", ppAuto());
  }

  public Command ppAuto() {
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem,
            new Pose2d(new Translation2d(1.92, 4.91), m_poseEstimationSubsystem.getPose().getRotation())),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("2coneA", 4, 3)));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.back().onTrue(new InstantCommand(() -> m_poseEstimationSubsystem.zeroGyro()));
    // m_driverController.x().onTrue(new SeekingCommand(m_visionSubsystem,
    // m_drivetrainSubsystem));
    m_driverController.b()
        .whileTrue(new TurnToAngleCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, Math.PI / 2.0));
    m_driverController.rightBumper().whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem,
        m_poseEstimationSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) * Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getLeftX(), false) * Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getRightX(), false) * MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis(),
        new Pose2d(new Translation2d(0, 0), new Rotation2d())));

    m_driverController.x().whileTrue(new GoToPoseCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        new Pose2d(new Translation2d(0, 0), new Rotation2d())));
}

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value, boolean squareAxis) {
    // Deadband
    value = deadband(value, 0.075);

    // Square the Axis
    if (squareAxis) {
      value = Math.copySign(value * value, value);
    }
    return value;
  }
}
