// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.CommandMacroPad;
import frc.lib.util.MacroPad;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmJoystickCommand;
import frc.robot.commands.ArmPositionCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CalibrateOdometryCommand;
import frc.robot.commands.ClawPowerCommand;
import frc.robot.commands.ClawTimedCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ExtendToCommand;
import frc.robot.commands.ExtensionJoystickCommand;
import frc.robot.commands.GoToPoseCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.GoToSubstationCommand;
import frc.robot.commands.LockedRotDriveCommand;
import frc.robot.commands.PPCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.WristCommand;
import frc.robot.commands.WristSwitchCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.nio.file.Path;
// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.function.DoubleSupplier;

import org.ejml.dense.row.MatrixFeatures_CDRM;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.Swerve.*;
import static frc.robot.Constants.ArmPosition.*;

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
  private final ClawSubsystem m_clawSubsystem = new ClawSubsystem();
  private final TargetingSubsystem m_targetingSubsystem = new TargetingSubsystem(
      new CommandMacroPad(OperatorConstants.kMacropadPort));
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  private final PoseEstimationSubsystem m_poseEstimationSubsystem = new PoseEstimationSubsystem(m_drivetrainSubsystem,
      m_visionSubsystem);
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ExtensionSubsystem m_extensionSubsystem = new ExtensionSubsystem();
  private final WristSubsystem m_wristSubsystem = new WristSubsystem();

  private SendableChooser<Command> m_autoChooser;

  private boolean isInDefaultDriveMode = true;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OperatorConstants.kOperatorControllerPort);

  private double rumble = 0;
  private DoubleSupplier armAngle;
  private DoubleSupplier telescopeLength;

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

    armAngle = () -> m_targetingSubsystem.getScoringHeight().rotation;

    telescopeLength = () -> m_targetingSubsystem.getScoringHeight().length;

    addAuton();
    SmartDashboard.putData(m_autoChooser);
    // Configure the trigger bindings
    setDefaultCommands();
    configureBindings();

  }

  public void setDefaultCommands() {

    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getLeftX(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis(),
        true));

    m_armSubsystem.setDefaultCommand(
        new ArmJoystickCommand(m_armSubsystem, () -> modifyAxis(m_operatorController.getRightY(), false)));
    m_extensionSubsystem.setDefaultCommand(
        new ExtensionJoystickCommand(m_extensionSubsystem, () -> modifyAxis(m_operatorController.getLeftY(), false)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void addAuton() {
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("PathplannerAuton", ppAuto());
  }

  public Command moveArm(ArmPosition position) {
    return new InstantCommand(
        () -> {
          m_armSubsystem.snapToAngle(position.rotation);
        }, m_armSubsystem)
        .alongWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(position.length);
            }, m_extensionSubsystem));
  }

  public Command ppAuto() {
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem,
            new Pose2d(new Translation2d(1.92, 4.91),
                m_poseEstimationSubsystem.getPose().getRotation())),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("path 1.1", 2.5, 2)));
  }

  /**
   * Auton scoring one cone and one cube high. Balances at the end.
   * <p>
   * Initialize this auton against the node closest to the loading zone.
   */
  public Command twoPieceLoadingSide() {

    PathPlannerTrajectory firstPath = PathPlanner.loadPath("path 1.1", 4.0, 3.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, firstPath.getInitialPose()),
        moveArm(HIGH).alongWith(waitCommand(1.5)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        moveArm(STOW),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, firstPath).alongWith(
            waitCommand(1.75).andThen(
                moveArm(LOW).alongWith(new ClawTimedCommand(m_clawSubsystem, 1, 0.6)))),
        moveArm(DEFAULT),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("path 1.2", 4.0, 3.0)),
        moveArm(HIGH).alongWith(waitCommand(1.5)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        moveArm(STOW),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("path 1.3", 0.8, 3.0)),
        waitCommand(1.37).deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command coneAndCubeLoadingSide() {
    PathPlannerTrajectory secondPath = PathPlanner.loadPath("path 2.1", 4.0, 3.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, secondPath.getInitialPose()),
        moveArm(HIGH).alongWith(waitCommand(1.5)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        moveArm(STOW),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, secondPath).alongWith(
            waitCommand(1.75).andThen(
                moveArm(LOW).alongWith(new ClawTimedCommand(m_clawSubsystem, 1, 0.6)))),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("path 2.2", 4.0, 3.0)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        moveArm(STOW),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("path 2.3", 0.8, 3.0)),
        waitCommand(1.37).deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command balancingLoadingSide() {
    PathPlannerTrajectory thirdPath = PathPlanner.loadPath("path 3.1", 0.8, 3.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, thirdPath.getInitialPose()),
        moveArm(HIGH).alongWith(waitCommand(1.5)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, thirdPath),
        waitCommand(1.37).deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command balancingCenterGrid() {
    PathPlannerTrajectory fourthPath = PathPlanner.loadPath("path 4.1", 0.8, 3.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, fourthPath.getInitialPose()),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, fourthPath),
        waitCommand(1.37).deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command onePieceCenterGrid() {
    PathPlannerTrajectory fifthPath = PathPlanner.loadPath("path 5.1", 2.0, 1.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, fifthPath.getInitialPose()),
        moveArm(HIGH).alongWith(waitCommand(1.5)),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        moveArm(LOW),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, fifthPath)
            .alongWith(waitCommand(2.5))
            .andThen(new ClawTimedCommand(m_clawSubsystem, 1, -0.6)),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 5.2", 2.0, 1.0)));
  }

  public Command threePiece() {
    PathPlannerTrajectory ninthPath = PathPlanner.loadPath("path 9.1", 4.0, 3.0);
    return new SequentialCommandGroup(
        new CalibrateOdometryCommand(m_poseEstimationSubsystem, ninthPath.getInitialPose()),
        moveArm(HIGH).alongWith(waitCommand(1.5)), // drops first piece
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, ninthPath) // moves to second piece and intakes
            .alongWith(moveArm(FLOOR))
            .alongWith(waitCommand(2)
                .andThen(new ClawTimedCommand(m_clawSubsystem, 1.2, 0.6))),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.2", 4.0, 3.0)) // moves
                                                                                                                    // to
                                                                                                                    // grid
            .alongWith(moveArm(HIGH))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6), // drops off second piece
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.3", 4.0, 3.0)) // moves
                                                                                                                    // and
                                                                                                                    // pick
                                                                                                                    // up
                                                                                                                    // third
                                                                                                                    // piece
            .alongWith(moveArm(FLOOR))
            .alongWith(waitCommand(2)
                .andThen(new ClawTimedCommand(m_clawSubsystem, 1, 0.6))),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.4", 4.0, 3.0)) // moves
                                                                                                                    // to
                                                                                                                    // grid
            .alongWith(moveArm(HIGH))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6), // drops third piece
        moveArm(DEFAULT));
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     */

    m_operatorController.b().whileTrue(new ClawPowerCommand(m_clawSubsystem, 1));
    m_operatorController.a().whileTrue(new ClawPowerCommand(m_clawSubsystem, -1));
    // m_operatorController.povLeft().whileTrue(new WristCommand(m_wristSubsystem,
    // 1));
    // m_operatorController.povRight().whileTrue(new WristCommand(m_wristSubsystem,
    // -1));

    // m_operatorController.y().onTrue(new WristSwitchCommand(m_wristSubsystem));
    // m_operatorController.a().whileTrue(new InstantCommand(() ->
    // m_wristSubsystem.rotateAt(-8)));
    m_operatorController.povDown().whileTrue(new ExtendToCommand(m_extensionSubsystem,
        0));
    // m_operatorController.b().whileTrue(new ExtendToCommand(m_extensionSubsystem,
    // 21.8));
    // m_operatorController.x().whileTrue(new ExtendToCommand(m_extensionSubsystem,
    // 19.5));
    m_operatorController.back().onTrue(m_armSubsystem.resetEncoder()
        .alongWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.resetEncoder();
            }, m_extensionSubsystem)));

    // m_operatorController.y().onTrue(
    // new InstantCommand(() -> {
    // m_wristSubsystem.rotate();
    // }));

    m_operatorController.x().whileTrue(new GoToSubstationCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)
        .alongWith((moveArm(DOUBLESUB))) // moves to double sub arm position
        .alongWith(new ClawPowerCommand(m_clawSubsystem, 1))) // intakes
        .onFalse(moveArm(DEFAULT)); // moves arm back to zero position

    m_operatorController.b().whileTrue(moveArm(ArmPosition.FLOOR) // moves arm to floor
        .alongWith(new ClawPowerCommand(m_clawSubsystem, 1))) // intakes
        .onFalse(moveArm(DEFAULT)); // moves arm back to zero position

    m_operatorController.povUp()
        .whileTrue((new GoToPoseCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, m_targetingSubsystem)) // Go
                                                                                                                 // to
                                                                                                                 // pose
            .alongWith(
                moveArm(m_targetingSubsystem.getScoringHeight()))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })) // rotate wristF
            .alongWith(waitCommand(2).andThen(new ClawTimedCommand(m_clawSubsystem, 1, -0.6)))) // Spit out
        .onFalse(moveArm(DEFAULT)); // move arm back

    m_operatorController.y().onTrue(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }));

    m_operatorController.rightBumper().whileTrue(
        new ArmJoystickCommand(m_armSubsystem, () -> modifyAxis(m_operatorController.getRightY(), false))
            .alongWith(
                new ExtensionJoystickCommand(m_extensionSubsystem,
                    () -> modifyAxis(m_operatorController.getLeftY(), false))));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      rumble = rumble == 0 ? 1 : 0;
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }));

    new Trigger(() -> isInDefaultDriveMode)
        .whileTrue(new DefaultDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            () -> -modifyAxis(m_driverController.getLeftY(), false) *
                Constants.Swerve.maxSpeed,
            () -> -modifyAxis(m_driverController.getLeftX(), false) *
                Constants.Swerve.maxSpeed,
            () -> -modifyAxis(m_driverController.getRightX(), false) *
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
            () -> m_driverController.getLeftTriggerAxis(),
            true))
        .whileFalse(
            new LockedRotDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                () -> -modifyAxis(m_driverController.getLeftY(), false) *
                    Constants.Swerve.maxSpeed,
                () -> -modifyAxis(m_driverController.getLeftX(), false) *
                    Constants.Swerve.maxSpeed,
                () -> -modifyAxis(m_driverController.getRightX(), false),
                () -> -modifyAxis(m_driverController.getRightY(), false),
                () -> -modifyAxis(m_driverController.getRightTriggerAxis(), false)));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    m_driverController.back().onTrue(new InstantCommand(() -> m_poseEstimationSubsystem.zeroGyro()));
    m_driverController.b()
        .whileTrue(new TurnToAngleCommand(m_drivetrainSubsystem,
            m_poseEstimationSubsystem, Math.PI / 2.0));

    m_driverController.rightBumper().whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem,
        m_poseEstimationSubsystem,
        m_targetingSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getLeftX(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis()));

    // m_driverController.rightBumper().whileTrue(new
    // GoToPoseCommand(m_drivetrainSubsystem,
    // m_poseEstimationSubsystem,
    // m_targetingSubsystem)
    // .alongWith(new SequentialCommandGroup(waitCommand(1),
    // new InstantCommand(() -> {
    // m_extensionSubsystem.extendTo(telescopeLength.getAsDouble());
    // }, m_extensionSubsystem)
    // .alongWith(new InstantCommand(() -> {
    // m_armSubsystem.snapToAngle(armAngle.getAsDouble());
    // }, m_armSubsystem)))));

    // m_driverController.a().onTrue(
    // (new SequentialCommandGroup(
    // new ClawTimedCommand(m_clawSubsystem, 1, -0.6),
    // waitCommand(0.5),
    // new InstantCommand(() ->
    // m_extensionSubsystem.extendTo(ArmPosition.DEFAULT.length))
    // .alongWith(new InstantCommand(() ->
    // m_armSubsystem.snapToAngle(ArmPosition.DEFAULT.rotation))))));

    // m_driverController.x().whileTrue(new GoToPoseCommand(m_drivetrainSubsystem,
    // m_poseEstimationSubsystem,
    // new Pose2d(new Translation2d(0, 0), new Rotation2d())));

    // m_driverController.y().whileTrue(new
    // AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem));

    // m_driverController.x().whileTrue(new GoToPoseCommand(m_drivetrainSubsystem,
    // m_poseEstimationSubsystem,
    // new Pose2d(new Translation2d(0, 0), new Rotation2d())));

    // m_operatorController.rightBumper().onTrue(new
    // ExampleCommand(m_targetingSubsystem));
  }

  private static Command waitCommand(double seconds) {
    return new WaitCommand(seconds);
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
