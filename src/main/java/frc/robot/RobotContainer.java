// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;
// import java.lang.invoke.ClassSpecializer.SpeciesData;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.CommandMacroPad;
import frc.robot.Constants.ArmPosition;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmJoystickCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.commands.CalibrateOdometryCommand;
import frc.robot.commands.ClawPowerCommand;
import frc.robot.commands.ClawTimedCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ExtensionJoystickCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.GoToSubstationCommand;
import frc.robot.commands.LEDSwitchColorsCommand;
import frc.robot.commands.LockedRotDriveCommand;
import frc.robot.commands.PPCommand;
import frc.robot.commands.TimerCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.WristSubsystem;

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
  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();

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

    // new Trigger(() -> m_armSubsystem.getArmAngle() >= 85).onTrue(new
    // WristCommand(m_wristSubsystem, true))
    // .onFalse(new WristCommand(m_wristSubsystem, false));

    // new Trigger(() -> (m_armSubsystem.getArmAngle() <= 85 &&
    // m_armSubsystem.getArmAngle() >= -50))
    // .whileTrue(new InstantCommand(() -> {
    // m_extensionSubsystem.extendTo(0);
    // }, m_extensionSubsystem));

    // Configure the trigger bindings
    setDefaultCommands();
    configureDriver1Bindings();
    configureDriver2Bindings();
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

    // m_armSubsystem.setDefaultCommand(
    // new ArmJoystickCommand(m_armSubsystem, () ->
    // modifyAxis(m_operatorController.getRightY(), false)));
    // m_extensionSubsystem.setDefaultCommand(
    // new ExtensionJoystickCommand(m_extensionSubsystem, () ->
    // modifyAxis(m_operatorController.getLeftY(), false)));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void addAuton() {
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("1 Cone + Straight FWD", oneConeAndTaxiAuto());
    m_autoChooser.addOption("1 Cone + Straight FWD", oneConeAndTaxiAuto());
    m_autoChooser.addOption("1 Cone ONLY", oneConeAuto());
    m_autoChooser.addOption("taxi", taxiAuto());
    m_autoChooser.addOption("1 cone FAR TAXI", oneConeTaxiFAR());
    m_autoChooser.addOption("JUN TEST AUTO", testAuto());
    m_autoChooser.addOption("super secret 2 cone", secret2Cone());
    m_autoChooser.addOption("back up climbing test 1", ppAuto());

    m_autoChooser.addOption("spin climbing test 2", spinClimb());

    m_autoChooser.addOption("secret 1 cone + climb", oneConeBalance());

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

  /**
   * Move arm to specified position, delaying the extension
   */
  public Command moveArm(ArmPosition position, double extensionDelay) {
    return new InstantCommand(
        () -> {
          m_armSubsystem.snapToAngle(position.rotation);
        }, m_armSubsystem)
        .alongWith(
            new WaitCommand(extensionDelay).andThen(
                new InstantCommand(() -> {
                  m_extensionSubsystem.extendTo(position.length);
                }, m_extensionSubsystem)));
  }

  public Command ppAuto() {
    return new SequentialCommandGroup(
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("climbPlz.1", 1.0, 1.0)),
        new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem));
  }

  public Command spinClimb() {
    return new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        PathPlanner.loadPath("SPINCLIMB", 4.0, 3.0));
  }

  /**
   * 1 cone then taxi
   */
  public Command oneConeAndTaxiAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGH, 0.5),
        new WaitCommand(1.5),
        moveArm(HIGHCOMMIT),
        new WaitCommand(2),
        new WaitCommand(3).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.25).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        moveArm(ZEROES),
        new WaitCommand(1.5),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 10.1", 4.0, 3.0)));
  }

  public Command oneConeBalance() {
    return new SequentialCommandGroup(
        moveArm(CUBENODEHIGH, 0.5),
        new WaitCommand(1.5),
        new WaitCommand(1.5).deadlineWith(
            new WaitCommand(0.25).andThen(
                new ClawPowerCommand(m_clawSubsystem, -1))),
        moveArm(ZEROES),
        new WaitCommand(1),
        moveArm(SALUTE),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("climbPlz.1", 1.0, 1.0)),
        new WaitCommand(5).deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command secret2Cone() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        // moveArm(HIGH, 0.5),
        new WaitCommand(1.5),
        // moveArm(HIGHCOMMIT),
        new WaitCommand(0.5),
        new WaitCommand(0.75).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.25).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        // moveArm(ZEROES),
        new WaitCommand(0.5),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 1.1", 4.0, 3.0))
            .alongWith(
                new ParallelCommandGroup(
                    new WaitCommand(0.5).andThen(
                      // moveArm(INTAKEBACK, 1)
                      ),
                    new WaitCommand(1)
                        .andThen(new WaitCommand(1.4).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 1.2", 4.0, 3.0))
            .alongWith(
              // moveArm(ZEROES)
              // .andThen
              (new WaitCommand(1.8)),
              // .andThen(moveArm(CUBENODEHIGH, 0.5)))),
        new ClawPowerCommand(m_clawSubsystem, -0.8)));
  }

  public Command oneConeTaxiFAR() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGH, 0.5),
        new WaitCommand(1.5),
        moveArm(HIGHCOMMIT),
        new WaitCommand(2),
        new WaitCommand(3).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.25).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        moveArm(ZEROES),
        new WaitCommand(1.5),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("taxi very far", 4.0, 3.0)));
  }

  public Command oneConeAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGH, 0.5),
        new WaitCommand(1.5),
        moveArm(HIGHCOMMIT),
        new WaitCommand(3),
        new WaitCommand(3).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.25).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        moveArm(ZEROES));
  }

  public Command testAuto() {
    return new SequentialCommandGroup(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        PathPlanner.loadPath("testingAuto", 4.0, 3.0)),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("testingAuto2", 4.0, 3.0)),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("testingAuto3", 4.0, 3.0)));
  }

  public Command taxiAuto() {
    return new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 10.1", 4.0, 3.0));
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
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.2", 4.0, 3.0))
            .alongWith(moveArm(HIGH))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6), // drops off second piece
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.3", 4.0, 3.0))
            .alongWith(moveArm(FLOOR))
            .alongWith(waitCommand(2)
                .andThen(new ClawTimedCommand(m_clawSubsystem, 1, 0.6))),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 9.4", 4.0, 3.0))
            .alongWith(moveArm(HIGH))
            .alongWith(new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            })),
        new ClawTimedCommand(m_clawSubsystem, 1, -0.6), // drops third piece
        moveArm(DEFAULT));
  }

  private void configureDriver2Bindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     */

    m_driverController.start().onTrue(new
    LEDSwitchColorsCommand(m_LEDSubsystem));

    m_operatorController.b().whileTrue(new ClawPowerCommand(m_clawSubsystem, 1));
    m_operatorController.a().whileTrue(new ClawPowerCommand(m_clawSubsystem, -1));
    // m_operatorController.povLeft().whileTrue(new WristCommand(m_wristSubsystem,
    // 1));
    // m_operatorController.povRight().whileTrue(new WristCommand(m_wristSubsystem,
    // -1));

    // m_operatorController.y().onTrue(new WristSwitchCommand(m_wristSubsystem));
    // m_operatorController.a().whileTrue(new InstantCommand(() ->
    // m_wristSubsystem.rotateAt(-8)));

    m_operatorController.povRight().onTrue(new InstantCommand(() -> {
      m_armSubsystem.snapToAngle(90);
    }, m_armSubsystem).alongWith(new InstantCommand(() -> {
      m_extensionSubsystem.extendTo(0);
    }, m_extensionSubsystem)));
    m_operatorController.povLeft().onTrue(new InstantCommand(() -> {
      m_extensionSubsystem.extendTo(0);
    }, m_extensionSubsystem));
    m_operatorController.povDown().onTrue(
        new InstantCommand(
            () -> {
              ArmPosition position = m_targetingSubsystem.getScoringHeight();
              m_armSubsystem.snapToAngle(position.rotation);
            }, m_armSubsystem)
            .alongWith(
                new InstantCommand(() -> {
                  ArmPosition position = m_targetingSubsystem.getScoringHeight();
                  m_extensionSubsystem.extendTo(position.length);
                }, m_extensionSubsystem)));
    // m_operatorController.b().whileTrue(new ExtendToCommand(m_extensionSubsystem,
    // 21.8));
    // m_operatorController.x().whileTrue(new ExtendToCommand(m_extensionSubsystem,
    // 19.5));
    // m_operatorController.back().onTrue(m_armSubsystem.resetEncoder(45)
    // .alongWith(
    // new InstantCommand(() -> {
    // m_extensionSubsystem.resetEncoder(0);
    // }, m_extensionSubsystem)));

    m_operatorController.back().onTrue(m_armSubsystem.resetEncoder(0)
        .alongWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.resetEncoder(0);
            }, m_extensionSubsystem)));

    m_driverController.rightStick().whileTrue(
        new InstantCommand(
            () -> {
              ArmPosition position = m_targetingSubsystem.getCommitedScoringHeight();
              m_armSubsystem.snapToAngle(position.rotation);
            }, m_armSubsystem)
            .alongWith(
                new InstantCommand(() -> {
                  ArmPosition position = m_targetingSubsystem.getCommitedScoringHeight();
                  m_extensionSubsystem.extendTo(position.length);
                }, m_extensionSubsystem))
            .alongWith(new TimerCommand(0.75)).andThen(
                new ClawPowerCommand(m_clawSubsystem, -1).alongWith(
                    new InstantCommand(() -> {
                      m_extensionSubsystem.extendTo(0);
                    }, m_extensionSubsystem))));

    m_operatorController.povUp().onTrue(moveArm(DOUBLESUB));

    m_operatorController.leftStick().onTrue(moveArm(SHOOTPOSITION));

    m_operatorController.rightTrigger().onTrue(moveArm(INTAKEBACK));

    m_operatorController.leftTrigger().onTrue(moveArm(INTAKEFRONT));

    m_operatorController.x().whileTrue(
        new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(19.3);
        }, m_extensionSubsystem).alongWith(new WaitCommand(0.5).andThen(
            new ClawPowerCommand(m_clawSubsystem, -(5 / 3)))));

    m_operatorController.y().onTrue(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }));

    m_operatorController.rightBumper().whileTrue(
        new ArmJoystickCommand(m_armSubsystem, () -> modifyAxis(m_operatorController.getRightY(), false)));

    m_operatorController.rightBumper().whileTrue(new ExtensionJoystickCommand(m_extensionSubsystem,
        () -> modifyAxis(m_operatorController.getLeftY(), false)));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      rumble = rumble == 0 ? 1 : 0;
      m_operatorController.getHID().setRumble(RumbleType.kBothRumble, rumble);
    }));

    m_operatorController.start().onTrue(moveArm(ZEROES));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.

    // m_operatorController.rightBumper().onTrue(new
    // ExampleCommand(m_targetingSubsystem));
  }

  private void configureDriver1Bindings() {

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

    m_driverController.leftBumper().whileTrue(new GoToSubstationCommand(m_drivetrainSubsystem,
        m_poseEstimationSubsystem,
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

    m_driverController.y().whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem));

    // m_driverController.x().whileTrue(new GoToPoseCommand(m_drivetrainSubsystem,
    // m_poseEstimationSubsystem,
    // new Pose2d(new Translation2d(0, 0), new Rotation2d())));
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
