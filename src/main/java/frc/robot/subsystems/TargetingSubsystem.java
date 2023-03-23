// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CommandMacroPad;
import frc.lib.util.MacroPad;
import frc.lib.util.MacroPad.Button;
import frc.robot.Constants.GridPositions;
import frc.robot.Constants.ArmPosition;

public class TargetingSubsystem extends SubsystemBase {

  private final double POLE_OFFSET = .555;

  int desiredGrid;

  boolean isBlueAlliance = true;

  private final CommandMacroPad m_pad;
  private Button desiredNode = Button.button1;

  public double armAngle = 0;
  public double elevatorLength = 0;
  public ArmPosition pos = ArmPosition.HIGH;
  public ArmPosition commitPos = ArmPosition.HIGHCOMMIT;
  public boolean isCone = false;

  /** Creates a new TargettingSubsystem. */
  public TargetingSubsystem(CommandMacroPad m_commandMacroPad) {
    m_pad = m_commandMacroPad;
    SmartDashboard.putBoolean("Alliance", false);
  }

  public Pose2d getTargetGridPose() {
    DriverStation.refreshData();
    boolean isRed = DriverStation.getAlliance().equals(Alliance.Red);
    int grid = isRed ? desiredGrid - 1: desiredGrid - 3;
    Translation2d offset = getNodeOffset();
    return new Pose2d(GridPositions.values()[grid].x + offset.getX(),
        GridPositions.values()[grid].y + offset.getY(),
        Rotation2d.fromDegrees(isBlueAlliance ? 0 : 180));
  }

  public void setNode(int node) {
    if (node >= 1 && node <= 9)
      desiredNode = Button.values()[node - 1];
    else
      return;
  }

  private Translation2d getNodeOffset() {
    if (desiredNode == null)
      return new Translation2d();

    int allianceReverse = isBlueAlliance ? 1 : -1;

    // double xOffset = desiredNode.row == 1 ? -0.26 * allianceReverse : 0;
    double xOffset = -0.25;
    double yOffset;

    switch (desiredNode.column) {
      case 1:
        yOffset = -POLE_OFFSET * allianceReverse;
        break;
      case 2:
        yOffset = 0;
        break;
      case 3:
        yOffset = POLE_OFFSET * allianceReverse;
        break;
      default:
        yOffset = 0;
      break;
    }
    return new Translation2d(xOffset, yOffset);

  }

  private int selectGrid() {
    var buttons = m_pad.getButtonsPressed();
    for (Button button : buttons) {
      if (button.row == 4) {
        SmartDashboard.putNumber("grid selected", button.column);
        return button.column;
      }
    }
    // only returns from here if no grid selected
    // 0 as default is probably fine
    return 1;
  }

  private Button selectNode() {
    var buttons = m_pad.getButtonsPressed();
    for (Button button : buttons) {
      if (button.row < 4) {
        SmartDashboard.putNumber("node selected", button.value());
        return button;
      }
    }
    // only returns from here if no grid selected
    // 0 as default is probably fine
    return Button.button1;
  }

  public ArmPosition getScoringHeight() {
    if (desiredNode == null) {
      return ArmPosition.ZEROES;
    }
    if(desiredNode.column == 2){
      switch (desiredNode.row) {
        case 1:
          return ArmPosition.CUBENODEHIGH;
        case 2:
          return ArmPosition.CUBENODEMEDIUM;
        case 3:
          return ArmPosition.INTAKEFRONT;
        default:
          return ArmPosition.ZEROES;
      }
    }
    switch (desiredNode.row) {
      case 1:
        return ArmPosition.HIGH;
      case 2:
        return ArmPosition.MEDIUM;
      case 3:
        return ArmPosition.INTAKEFRONT;
      default:
        return ArmPosition.ZEROES;
    }
  }

  public ArmPosition getCommitedScoringHeight(){
    if (desiredNode == null) {
      return ArmPosition.ZEROES;
    }
    switch (desiredNode.row) {
      case 1:
        return ArmPosition.HIGHCOMMIT;
      case 2:
        return ArmPosition.MEDIUMCOMMIT;
      case 3:
        return ArmPosition.INTAKEFRONT;
      default:
        return ArmPosition.ZEROES;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // isBlueAlliance = SmartDashboard.getBoolean("alliance", true)
    // DriverStation.refreshData();
    isBlueAlliance = DriverStation.getAlliance().equals(Alliance.Blue);

    desiredGrid = selectGrid() + (isBlueAlliance ? 5 : 0);

    armAngle = getScoringHeight().rotation;
    elevatorLength = getScoringHeight().length;

    isCone = desiredNode.column != 2;
    setNode(selectNode().value());
    SmartDashboard.putNumber("grid selected", desiredGrid);
    SmartDashboard.putNumber("node selected", desiredNode.value());
    SmartDashboard.putNumber("desiredX", getTargetGridPose().getX());
    SmartDashboard.putNumber("desiredY", getTargetGridPose().getY());
    SmartDashboard.putNumber("desiredRotation", getTargetGridPose().getRotation().getDegrees());

    SmartDashboard.putNumber("EXTEND PLS", armAngle);
    SmartDashboard.putNumber("ROT PLS", elevatorLength);

    pos = getScoringHeight();
    commitPos = getCommitedScoringHeight();
  }
}
