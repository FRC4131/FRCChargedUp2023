// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ArmConstants.DEFAULT_MAX_ACCEL;
import static frc.robot.Constants.ArmConstants.DEFAULT_MAX_VELOCITY;
import static frc.robot.Constants.ArmConstants.FAST_MAX_ACCEL;
import static frc.robot.Constants.ArmConstants.FAST_MAX_VELOCITY;
import static frc.robot.Constants.ArmPosition.AUTONCUBEHIGH;
import static frc.robot.Constants.ArmPosition.CUBENODEHIGH;
import static frc.robot.Constants.ArmPosition.DEFAULT;
import static frc.robot.Constants.ArmPosition.FLOOR;
import static frc.robot.Constants.ArmPosition.HIGH;
import static frc.robot.Constants.ArmPosition.HIGHCOMMIT;
import static frc.robot.Constants.ArmPosition.INTAKEFRONT;
import static frc.robot.Constants.ArmPosition.LOW;
import static frc.robot.Constants.ArmPosition.SALUTE;
import static frc.robot.Constants.ArmPosition.SHOOTPOSITION;
import static frc.robot.Constants.ArmPosition.STOW;
import static frc.robot.Constants.ArmPosition.ZEROES;
import static frc.robot.Constants.Swerve.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
import static frc.robot.Constants.Swerve.MAX_VELOCITY_METERS_PER_SECOND;

import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
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
import frc.robot.commands.EmergencyDrive;
import frc.robot.commands.ExtensionJoystickCommand;
import frc.robot.commands.GoToPoseTeleopCommand;
import frc.robot.commands.NewAutoBalanceCommand;
import frc.robot.commands.PPCommand;
import frc.robot.commands.RampingDriveCommand;
import frc.robot.commands.WristCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ExtensionSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;
import frc.robot.subsystems.TargetingSubsystem;
import frc.robot.subsystems.VisionSubsystemLL3;
import frc.robot.subsystems.WristSubsystem;

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
  private final VisionSubsystemLL3 m_visionSubsystem = new VisionSubsystemLL3();
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
  private DoubleSupplier commitAngle;
  private DoubleSupplier commitLength;
  private BooleanSupplier isCone;
  private DoubleSupplier currentArmAngle;

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

    armAngle = () -> m_targetingSubsystem.armAngle;
    telescopeLength = () -> m_targetingSubsystem.elevatorLength;
    commitAngle = () -> m_targetingSubsystem.commitPos.rotation;
    commitLength = () -> m_targetingSubsystem.commitPos.length;
    isCone = () -> m_targetingSubsystem.isCone;
    currentArmAngle = () -> m_armSubsystem.getArmAngle();
    addAuton();
    SmartDashboard.putData(m_autoChooser);

    // Configure the trigger bindings
    setDefaultCommands();
    configureDriver1Bindings();
    configureCompOperatorBindings();

    // Only for debugging.
    // configureDriver2Bindings();

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

    // new Trigger(isCone).onTrue(new InstantCommand(() -> {
    // m_LEDSubsystem.setHSV(-1, 170, 255, 255);
    // })).onFalse(new InstantCommand(() -> {
    // m_LEDSubsystem.setHSV(-1, 35, 255, 255);
    // }));

    new Trigger(isCone).onTrue(new RepeatCommand(new InstantCommand(() -> {
      m_LEDSubsystem.pulse(170);
    }, m_LEDSubsystem))).onFalse(new RepeatCommand(new InstantCommand(() -> {
      m_LEDSubsystem.pulse(35);
    }, m_LEDSubsystem)));

  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  public void addAuton() {
    m_autoChooser = new SendableChooser<Command>();
    m_autoChooser.setDefaultOption("MIDDLE Cube + Balance", dcmpAuto1());
    m_autoChooser.addOption("MIDDLE Cube + Balance", dcmpAuto1());
    m_autoChooser.addOption("OPEN SIDE 2 Cube + Balance", dcmpAuto2());
    m_autoChooser.addOption("CABLE SIDE 2.5 Cube + Balance", dcmpAuto3());
    m_autoChooser.addOption("NO BALANCE middle Cube + Balance", dcmpAuto1NOBAL());
    m_autoChooser.addOption("NO BALANCE open side 2 Cube + Balance", dcmpAuto2NOBAL());
    m_autoChooser.addOption("NO BALANCE cable side 2.5 Cube + Balance", dcmpAuto3NOBAL());
    m_autoChooser.addOption("just taxi from grid to cube", taxiAuto());
    m_autoChooser.addOption("1 Cube + balance NO TAXI", oneCubeBalance());
    m_autoChooser.addOption("1 Cone + taxi", oneConeAndTaxiAuto());
    m_autoChooser.addOption("1 Cone ONLY", oneConeAuto());
    m_autoChooser.addOption("1 Cone FAR TAXI", oneConeTaxiFAR());

    // m_autoChooser.addOption("Y axis test",
    // new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
    // PathPlanner.loadPath("climbPlz.1", 1.0, 1.0)));

    // m_autoChooser.addOption("2 piece + balance TESTED", thisWorky());
    // m_autoChooser.addOption("UNTESTED 2 piece + balance slightly less spacey",
    // goodAutoLessSpace());
    // m_autoChooser.addOption("2 piece cable ONLY", twoPieceCable());
    // m_autoChooser.addOption("UNTESTED just 1 cone + balance", slowTwoPiece());
    // m_autoChooser.addOption("TESTING AUTO", new SequentialCommandGroup(
    // new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
    // PathPlanner.loadPath("dcmp1.1", 2.0, 1.0)),
    // new WaitCommand(15).deadlineWith(
    // new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
    // false))));
  }

  public Command thisWorky() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.6),
        new WaitCommand(0.8),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("twoPieceCable.1", 4.0, 3.0))
                .andThen(new WaitCommand(0.7))
                .andThen(
                    new WaitCommand(2.48).deadlineWith(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                        PathPlanner.loadPath("twoPieceCable.2", 3.0, 3.0))))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false)),
            moveArm(SALUTE, true)
                .andThen(new WaitCommand(1))
                .andThen(moveArm(INTAKEFRONT))
                .andThen(new WaitCommand(3))
                .andThen(moveArm(SALUTE))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(AUTONCUBEHIGH))
                .andThen(new WaitCommand(2.2))
                .andThen(moveArm(SALUTE)),
            new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            }),
            new WaitCommand(1)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2.8).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 0.1)))));
    // .andThen(new WaitCommand(0.8).deadlineWith(new
    // ClawPowerCommand(m_clawSubsystem, -0.8)))));
  }

  public Command goodAutoLessSpace() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.6),
        new WaitCommand(0.8),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("twoPieceCable.1", 4.0, 3.0))
                .andThen(new WaitCommand(0.7))
                .andThen(
                    new WaitCommand(2.48).deadlineWith(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                        PathPlanner.loadPath("twoPieceCable.3", 3.0, 3.0))))
                .andThen(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false)),
            moveArm(SALUTE, true)
                .andThen(new WaitCommand(1))
                .andThen(moveArm(INTAKEFRONT))
                .andThen(new WaitCommand(3))
                .andThen(moveArm(SALUTE))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(AUTONCUBEHIGH))
                .andThen(new WaitCommand(2.2))
                .andThen(moveArm(SALUTE)),
            new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            }),
            new WaitCommand(1)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2.8).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 0.1)))));
    // .andThen(new WaitCommand(0.8).deadlineWith(new
    // ClawPowerCommand(m_clawSubsystem, -0.8)))));
  }

  public Command twoPieceCable() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.6),
        new WaitCommand(0.8),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("twoPieceCable.1", 4.0, 3.0)),
            moveArm(SALUTE, true)
                .andThen(new WaitCommand(1))
                .andThen(moveArm(ArmPosition.INTAKEFRONT))
                .andThen(new WaitCommand(3))
                .andThen(moveArm(SALUTE))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(AUTONCUBEHIGH))
                .andThen(new WaitCommand(2.2))
                .andThen(moveArm(SALUTE)),
            new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            }),
            new WaitCommand(1)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2.8).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 0.1)))));
    // .andThen(new WaitCommand(0.8).deadlineWith(new
    // ClawPowerCommand(m_clawSubsystem, -0.8)))));
  }

  public Command slowTwoPiece() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.6),
        new WaitCommand(0.8),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("twoPieceCable.4", 4.0, 2.25)),
            moveArm(SALUTE, true)
                .andThen(new WaitCommand(1))
                .andThen(moveArm(INTAKEFRONT))
                .andThen(new WaitCommand(3))
                .andThen(moveArm(SALUTE))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(AUTONCUBEHIGH))
                .andThen(new WaitCommand(2.2))
                .andThen(moveArm(SALUTE)),
            new InstantCommand(() -> {
              m_wristSubsystem.rotate();
            }),
            new WaitCommand(1)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2.8).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 0.1)))));
    // .andThen(new WaitCommand(0.8).deadlineWith(new
    // ClawPowerCommand(m_clawSubsystem, -0.8)))));
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

  public Command moveArm(ArmPosition position, double extensionDelay, boolean fast) {
    return new InstantCommand(
        () -> {
          m_armSubsystem.adjustSpeed(fast ? FAST_MAX_VELOCITY : DEFAULT_MAX_VELOCITY,
              fast ? FAST_MAX_ACCEL : DEFAULT_MAX_ACCEL);
          m_armSubsystem.clampSpeed(fast ? 1 : 0.5);
          m_armSubsystem.snapToAngle(position.rotation);
        }, m_armSubsystem)
        .alongWith(
            new WaitCommand(extensionDelay).andThen(
                new InstantCommand(() -> {
                  m_extensionSubsystem.extendTo(position.length);
                }, m_extensionSubsystem)));
  }

  public Command moveArm(ArmPosition position, boolean fast) {
    return new InstantCommand(
        () -> {
          m_armSubsystem.adjustSpeed(fast ? FAST_MAX_VELOCITY : DEFAULT_MAX_VELOCITY,
              fast ? FAST_MAX_ACCEL : DEFAULT_MAX_ACCEL);
          m_armSubsystem.clampSpeed(fast ? 1 : 0.5);
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
          m_armSubsystem.adjustSpeed(DEFAULT_MAX_VELOCITY,
              DEFAULT_MAX_ACCEL);
          m_armSubsystem.clampSpeed(1);
          m_armSubsystem.snapToAngle(position.rotation);
        }, m_armSubsystem)
        .alongWith(
            new WaitCommand(extensionDelay).andThen(
                new InstantCommand(() -> {
                  m_extensionSubsystem.extendTo(position.length);
                }, m_extensionSubsystem)));
  }

  /**
   * The 2 piece + balance on cable side
   */
  public Command owww() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("OWWW", 4.0, 3.0))
                .andThen(new WaitCommand(2)
                    .deadlineWith(
                        new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false))),
            new WaitCommand(1.8).andThen(moveArm(ArmPosition.INTAKEBACK, false))
                .andThen(new WaitCommand(4))
                .andThen(moveArm(CUBENODEHIGH, 3.0, false))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(SALUTE)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(4))
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1)))));
  }

  /**
   * The 2 piece + balance on cable side
   */
  public Command owTests() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new WaitCommand(1.8).andThen(moveArm(ArmPosition.INTAKEBACK, false))
                .andThen(new WaitCommand(4))
                .andThen(moveArm(CUBENODEHIGH, 3.0, false))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(SALUTE)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(4))
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1)))));
  }

  public Command threePieceAuto() {
    return
    // new SequentialCommandGroup(
    // new InstantCommand(() -> {
    // m_wristSubsystem.rotate();
    // }),
    // moveArm(HIGHCOMMIT, 0.4),
    // new WaitCommand(1),
    // new WaitCommand(0.5).deadlineWith(
    // new InstantCommand(() -> {
    // m_extensionSubsystem.extendTo(0);
    // }, m_extensionSubsystem).alongWith(
    // new WaitCommand(0.1).andThen(
    // new ClawPowerCommand(m_clawSubsystem, -1)))),
    new ParallelCommandGroup(
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("loadSide3Piece", 4.0, 3.0)),
        new InstantCommand(() -> {
          m_armSubsystem.snapToAngle(ArmPosition.INTAKEFRONT.rotation);
        }, m_armSubsystem).alongWith(new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(ArmPosition.INTAKEFRONT.length);
        }, m_extensionSubsystem)),
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        // .andThen(new WaitCommand(4))
        // .andThen(moveArm(CUBENODEHIGH, 3.0, false))
        // .andThen(new WaitCommand(1.5))
        // .andThen(moveArm(SALUTE)),
        new WaitCommand(1.5)
            .andThen(new WaitCommand(2).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
            .andThen(new WaitCommand(2))
            .andThen(new WaitCommand(2).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1))));
  }

  public Command seeeeecret() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new WristCommand(m_wristSubsystem, true).alongWith(
            moveArm(STOW)).andThen(
                new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("CABLESECRET", 4.0, 3.0)))
            .alongWith(
                new SequentialCommandGroup(
                    new WaitCommand(2)

                )));
  }

  /**
   * 1 cone then taxi
   */
  public Command oneConeAndTaxiAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        new WaitCommand(0.5),
        moveArm(HIGH, 0.2),
        new WaitCommand(0.7),
        moveArm(HIGHCOMMIT),
        new WaitCommand(1),
        new WaitCommand(1).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.25).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        moveArm(SALUTE),
        new WaitCommand(1.5),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("path 10.1", 4.0, 3.0)));
  }

  public Command oneCubeBalance() {
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
        new WaitCommand(5)
            .deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false)));
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
            PathPlanner.loadPath("the taxi", 2.0, 3.0)));
  }

  public Command twoPieceLoadingAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.6),
        new WaitCommand(0.8),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new WaitCommand(1)
                .andThen(new InstantCommand(() -> {
                  m_wristSubsystem.rotate();
                }))
                .andThen(new WaitCommand(3.22))
                .andThen(new InstantCommand(() -> {
                  m_wristSubsystem.rotate();
                })),
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("path 2.1", 2.0, 2.0))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("path 2.2", 4.0, 3.0))),
            moveArm(SALUTE)
                .andThen(new WaitCommand(1))
                .andThen(moveArm(ArmPosition.INTAKEFRONTTELEOP))
                .andThen(new WaitCommand(2.22))
                .andThen(moveArm(SALUTE))
                .andThen(new WaitCommand(2.78)),
            new WaitCommand(1)
                .andThen(new WaitCommand(1.72).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))));
  }

  /**
   * Cycles pre loaded cone high, no driving
   */
  public Command oneConeAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }).alongWith(
            moveArm(SALUTE)));
  }

  public Command coneBalance() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }).alongWith(
            moveArm(SALUTE)),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
            PathPlanner.loadPath("climbPlz.1", 1.0, 1.0)),
        new WaitCommand(8.0)
            .deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false)));
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
    return new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("the taxi", 2.0, 3.0));
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
        waitCommand(1.37)
            .deadlineWith(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false)));
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

  /**
   * Controller bindings currently used at competitions for the operator.
   */
  private void configureCompOperatorBindings() {

    /*
     * Bindings for preparing to score.
     * Half press moves the arm to the desired angle
     * Full press extends the elevator to the desired position. Only runs if arm is
     * past the halfway point.
     * Arm will run back to default position once trigger is not pressed.
     */
    new Trigger(
        () -> m_operatorController.getRightTriggerAxis() > 0.2 && m_operatorController.getRightTriggerAxis() < 0.9)
        .onTrue(
            new InstantCommand(() -> {
              m_armSubsystem.adjustSpeed(DEFAULT_MAX_VELOCITY, DEFAULT_MAX_ACCEL);
              m_armSubsystem.clampSpeed(1);
              ArmPosition position = m_targetingSubsystem.getScoringHeight();
              m_armSubsystem.snapToAngle(position.rotation);
            }, m_armSubsystem).alongWith(new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem)));

    new Trigger(() -> m_operatorController.getRightTriggerAxis() >= 0.9 && currentArmAngle.getAsDouble() <= 0).onTrue(
        new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(m_targetingSubsystem.getScoringHeight().length);
        }, m_extensionSubsystem)).onFalse(new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(0);
        }, m_extensionSubsystem));

    new Trigger(() -> m_operatorController.getRightTriggerAxis() < 0.2).onTrue(
        new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(SALUTE.length);
        }).alongWith(new InstantCommand(() -> {
          m_armSubsystem.adjustSpeed(FAST_MAX_VELOCITY, FAST_MAX_ACCEL);
          m_armSubsystem.clampSpeed(1);
          m_armSubsystem.snapToAngle(SALUTE.rotation);
        }, m_armSubsystem)));

    /*
     * Bindings for intaking
     * 
     * LT half press moves arm to the right position
     * Full press activates intake
     * 
     * A-Intake from front
     * B-Intake from back
     */
    new Trigger(
        () -> m_operatorController.getLeftTriggerAxis() > 0.2 && m_operatorController.getLeftTriggerAxis() < 0.9)
        .onTrue(
            moveArm(ArmPosition.DOUBLESUB, false));

    new Trigger(() -> m_operatorController.getLeftTriggerAxis() >= 0.9).whileTrue(
        moveArm(ArmPosition.DOUBLESUB, true).alongWith(new ClawPowerCommand(m_clawSubsystem, 1)));

    new Trigger(() -> m_operatorController.getLeftTriggerAxis() < 0.2).onTrue(
        new InstantCommand(() -> {
          m_extensionSubsystem.extendTo(SALUTE.length);
        }).alongWith(new InstantCommand(() -> {
          m_armSubsystem.adjustSpeed(FAST_MAX_VELOCITY, FAST_MAX_ACCEL);
          m_armSubsystem.snapToAngle(SALUTE.rotation);
        }, m_armSubsystem)));

    m_operatorController.a()
        .whileTrue(moveArm(ArmPosition.INTAKEFRONTTELEOP, true).alongWith(new ClawPowerCommand(m_clawSubsystem, 1)));

    m_operatorController.a().onFalse(moveArm(SALUTE, 0.2, true)
        .alongWith(new InstantCommand(() -> {
          m_wristSubsystem.forceCCW();
        })));

    m_operatorController.b()
        .whileTrue(moveArm(ArmPosition.INTAKEBACK, true).alongWith(new ClawPowerCommand(m_clawSubsystem, 1)));

    m_operatorController.b().onFalse(moveArm(SALUTE, true));

    // Click left stick to move arm to the shoot position.
    m_operatorController.leftStick().onTrue(moveArm(SHOOTPOSITION));

    m_operatorController.x().whileTrue(
        new ClawPowerCommand(m_clawSubsystem, -(5.0 / 3.0)));

    m_operatorController.y().onTrue(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }));

    m_operatorController.rightBumper().whileTrue(
        new ArmJoystickCommand(m_armSubsystem, () -> modifyAxis(m_operatorController.getRightY(), false)));

    m_operatorController.rightBumper().whileTrue(new ExtensionJoystickCommand(m_extensionSubsystem,
        () -> modifyAxis(m_operatorController.getLeftY(), false)));

    m_operatorController.povLeft().whileTrue(new ClawPowerCommand(m_clawSubsystem, 1));
    m_operatorController.povRight().whileTrue(new ClawPowerCommand(m_clawSubsystem, -1));

    m_operatorController.povUp().onTrue(new InstantCommand(() -> {
      m_armSubsystem.resetPosition(currentArmAngle.getAsDouble() + 0.5);
    }));

    m_operatorController.povDown().onTrue(new InstantCommand(() -> {
      m_armSubsystem.resetPosition(currentArmAngle.getAsDouble() - 0.5);
    }));

    m_operatorController.leftBumper().onTrue(new InstantCommand(() -> {
      m_wristSubsystem.forceCCW();
    }));

    m_operatorController.back().onTrue(moveArm(SALUTE));
    m_operatorController.start()
        .whileTrue(moveArm(ArmPosition.SINGLESUB, true).alongWith(new ClawPowerCommand(m_clawSubsystem, 1)));

  }

  private void configureDriver2Bindings() {
    // m_operatorController.a().onTrue(moveArm(ArmPosition.MEDIUM));
    // m_operatorController.b().onTrue(moveArm(SALUTE));
    // m_operatorController.x().onTrue(moveArm(ArmPosition.INTAKEBACK));
    // m_operatorController.y().onTrue(moveArm(ZEROES));
    // m_operatorController.povUp().onTrue(moveArm(INTAKEFRONT));
    // m_operatorController.povDown().onTrue(new InstantCommand(() ->
    // {m_extensionSubsystem.extendTo(10);}));
    // m_operatorController.rightBumper().whileTrue(
    // new ArmJoystickCommand(m_armSubsystem, () ->
    // modifyAxis(m_operatorController.getRightY(), false)));

    // m_operatorController.rightBumper().whileTrue(new
    // ExtensionJoystickCommand(m_extensionSubsystem,
    // () -> modifyAxis(m_operatorController.getLeftY(), false)));
  }

  private void configureDriver1Bindings() {

    m_driverController.start().onTrue(new InstantCommand(() -> {
      m_LEDSubsystem.setHSV(-1, 130, 255, 255);
    }));
    m_driverController.back().onTrue(new InstantCommand(() -> {
      m_LEDSubsystem.setHSV(-1, 65, 255, 255);
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
        // .whileFalse(
        //     new EmergencyDrive(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        //         () -> -modifyAxis(m_driverController.getLeftY(), false) *
        //             Constants.Swerve.maxSpeed,
        //         () -> -modifyAxis(m_driverController.getLeftX(), false) *
        //             Constants.Swerve.maxSpeed,
        //         () -> -modifyAxis(m_driverController.getRightX(), false) *
        //             MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        //         () -> m_driverController.getLeftTriggerAxis(),
        //         true))
                ;

    m_driverController.back().onTrue(new InstantCommand(() -> m_poseEstimationSubsystem.zeroGyro()));

    m_driverController.x()
        .onTrue(new InstantCommand(() -> {
          isInDefaultDriveMode = isInDefaultDriveMode ? false : true;
          SmartDashboard.putBoolean("EMERGENCY DRIVE ON", !isInDefaultDriveMode);
        }));

    m_driverController.leftBumper().whileTrue(new GoToPoseTeleopCommand(m_drivetrainSubsystem,
        m_poseEstimationSubsystem,
        m_targetingSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getLeftX(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis()));

    m_driverController.rightBumper().whileTrue(new RampingDriveCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
        () -> -modifyAxis(m_driverController.getLeftY(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getLeftX(), false) *
            Constants.Swerve.maxSpeed,
        () -> -modifyAxis(m_driverController.getRightX(), false) *
            MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
        () -> m_driverController.getLeftTriggerAxis(),
        true));

    new Trigger(
        () -> m_driverController.getRightTriggerAxis() > 0.1 && m_driverController.getRightTriggerAxis() < 0.9)
        .onTrue(
            new InstantCommand(
                () -> {
                  m_armSubsystem.snapToAngle(commitAngle.getAsDouble());
                  SmartDashboard.putNumber("COMMIT ANG", commitAngle.getAsDouble());
                }, m_armSubsystem));

    new Trigger(() -> m_driverController.getRightTriggerAxis() >= 0.9).whileTrue(
        new ClawPowerCommand(m_clawSubsystem, -1).alongWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem)));

    m_driverController.povUp().onTrue(new InstantCommand(() -> {
      m_LEDSubsystem.setHSV(-1, 125, 255, 255);
    }));

    m_driverController.povDown().onTrue(new InstantCommand(() -> {
      m_LEDSubsystem.setHSV(-1, 60, 255, 255);
    }));

    m_driverController.povRight().toggleOnTrue(new RepeatCommand(new InstantCommand(() -> {
      m_LEDSubsystem.rainbow();
    })));

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

    m_driverController.y().whileTrue(new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, true));
    m_driverController.b().whileTrue(new NewAutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem));

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

  // im putting all the autons written after sundome down here because i hate this
  // file's organization

  public Command templateAuto() {
    return new SequentialCommandGroup(
        new InstantCommand(() -> {
          m_wristSubsystem.rotate();
        }),
        moveArm(HIGHCOMMIT, 0.4),
        new WaitCommand(1),
        new WaitCommand(0.5).deadlineWith(
            new InstantCommand(() -> {
              m_extensionSubsystem.extendTo(0);
            }, m_extensionSubsystem).alongWith(
                new WaitCommand(0.1).andThen(
                    new ClawPowerCommand(m_clawSubsystem, -1)))),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("OWWW", 4.0, 3.0))
                .andThen(new WaitCommand(2)
                    .deadlineWith(
                        new AutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, false))),
            new WaitCommand(1.8).andThen(moveArm(ArmPosition.INTAKEBACK, false))
                .andThen(new WaitCommand(4))
                .andThen(moveArm(CUBENODEHIGH, 3.0, false))
                .andThen(new WaitCommand(1.5))
                .andThen(moveArm(SALUTE)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(4))
                .andThen(new WaitCommand(3).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1)))));
  }

  /**
   * @return DCMP auto #1
   *         <p>
   *         Scores one cube mid, taxi, and then balance
   */
  public Command dcmpAuto1() {
    return new SequentialCommandGroup(
        moveArm(CUBENODEHIGH, 0.5),
        new WaitCommand(1),
        new WaitCommand(1).deadlineWith(
            new WaitCommand(0.25).andThen(
                new ClawPowerCommand(m_clawSubsystem, -1))),
        moveArm(ZEROES),
        new WaitCommand(1),
        moveArm(SALUTE),
        new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem, PathPlanner.loadPath("dcmp1.1", 2.0, 1.0)),
        new WaitCommand(5)
            .deadlineWith(new NewAutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem)));
  }

  public Command dcmpAuto1NOBAL() {
    return new SequentialCommandGroup(
        moveArm(CUBENODEHIGH, 0.5),
        new WaitCommand(1),
        new WaitCommand(1).deadlineWith(
            new WaitCommand(0.25).andThen(
                new ClawPowerCommand(m_clawSubsystem, -1))),
        moveArm(ZEROES),
        new WaitCommand(1),
        moveArm(SALUTE));
  }

  /**
   * @return DCMP auto #2
   *         <p>
   *         Scores one cube open side, grab + score another, and then balance in
   *         community
   */
  public Command dcmpAuto2() {
    return new SequentialCommandGroup(
        moveArm(ZEROES, true),
        new WaitCommand(0.5),
        moveArm(ArmPosition.FRONTCUBEHIGH, true),
        new WaitCommand(0.5),
        new WaitCommand(0.25).deadlineWith(
            new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))),
        moveArm(ArmPosition.PRE_FRONTCUBEHIGH, true),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("dcmp2.1", 4.0, 3.0))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("dcmp2.2", 4.0, 3.0)))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("dcmp2.3", 2.0, 2.0)))
                .andThen(new WaitCommand(4)
                    .deadlineWith(
                        new NewAutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem))),
            new WaitCommand(0.5).andThen(moveArm(ArmPosition.INTAKEBACK, true))
                .andThen(new WaitCommand(1.9))
                .andThen(moveArm(SALUTE, true)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(1.4).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2))
                .andThen(new WaitCommand(1).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1)))));
  }

  public Command dcmpAuto2NOBAL() {
    return new SequentialCommandGroup(
        moveArm(ZEROES, true),
        new WaitCommand(0.5),
        moveArm(ArmPosition.FRONTCUBEHIGH, true),
        new WaitCommand(0.5),
        new WaitCommand(0.25).deadlineWith(
            new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))),
        moveArm(ArmPosition.PRE_FRONTCUBEHIGH, true),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("dcmp2.1", 4.0, 3.0))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("dcmp2.2", 4.0, 3.0))),
            new WaitCommand(0.5).andThen(moveArm(ArmPosition.INTAKEBACK, true))
                .andThen(new WaitCommand(1.9))
                .andThen(moveArm(SALUTE, true)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(1.4).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(2))
                .andThen(new WaitCommand(1).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -1)))));
  }

  /**
   * @return DCMP auto #3
   *         <p>
   *         Scores 2 cubes mid, grabs another, then balances outside.
   */
  public Command dcmpAuto3() {
    return new SequentialCommandGroup(
        moveArm(ZEROES, true),
        new WaitCommand(0.5),
        moveArm(ArmPosition.FRONTCUBEHIGH, true),
        new WaitCommand(0.5),
        new WaitCommand(0.25).deadlineWith(
            new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))),
        moveArm(ArmPosition.PRE_FRONTCUBEHIGH, true),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("dcmp3.1", 2.0, 3.0))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("dcmp3.2", 4.0, 3.0)))
                .andThen(new WaitCommand(4)
                    .deadlineWith(
                        new NewAutoBalanceCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem))),
            new WaitCommand(0.5).andThen(moveArm(ArmPosition.INTAKEBACK, true))
                .andThen(new WaitCommand(2.5))
                .andThen(moveArm(ArmPosition.MIDCUBEFRONT, true))
                .andThen(new WaitCommand(3.5))
                .andThen(moveArm(ArmPosition.INTAKEBACKCUBESLIGHTLYLOWER, true))
                .andThen(new WaitCommand(2.7))
                .andThen(moveArm(ArmPosition.MIDCUBEFRONT, true)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(1.5).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(3))
                .andThen(new WaitCommand(0.5).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))))
                .andThen(new WaitCommand(2))
                .andThen(new WaitCommand(3.2).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))));
  }

  public Command dcmpAuto3NOBAL() {
    return new SequentialCommandGroup(
        moveArm(ZEROES, true),
        new WaitCommand(0.5),
        moveArm(ArmPosition.FRONTCUBEHIGH, true),
        new WaitCommand(0.5),
        new WaitCommand(0.25).deadlineWith(
            new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))),
        moveArm(ArmPosition.PRE_FRONTCUBEHIGH, true),
        new ParallelCommandGroup(
            new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                PathPlanner.loadPath("dcmp3.1", 2.0, 3.0))
                .andThen(new PPCommand(m_drivetrainSubsystem, m_poseEstimationSubsystem,
                    PathPlanner.loadPath("dcmp3.2.2", 2.0, 3.0))),
            new WaitCommand(0.5).andThen(moveArm(ArmPosition.INTAKEBACK, true))
                .andThen(new WaitCommand(2.5))
                .andThen(moveArm(ArmPosition.MIDCUBEFRONT, true))
                .andThen(new WaitCommand(3.5))
                .andThen(moveArm(ArmPosition.INTAKEBACKCUBESLIGHTLYLOWER, true))
                .andThen(new WaitCommand(2.7))
                .andThen(moveArm(ArmPosition.MIDCUBEFRONT, true)),
            new WaitCommand(1.5)
                .andThen(new WaitCommand(1.5).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))
                .andThen(new WaitCommand(3))
                .andThen(new WaitCommand(0.5).deadlineWith(new ClawPowerCommand(m_clawSubsystem, -(3.0 / 5.0))))
                .andThen(new WaitCommand(2))
                .andThen(new WaitCommand(3.2).deadlineWith(new ClawPowerCommand(m_clawSubsystem, 1)))));
  }
}
