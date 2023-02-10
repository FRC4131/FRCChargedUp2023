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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.CommandMacroPad;
import frc.lib.util.MacroPad;
import frc.lib.util.MacroPad.Button;

public class TargetingSubsystem extends SubsystemBase {

  private final double POLE_OFFSET = .5;

  // Grid Pose numbers correspond to April Tag IDs
  // Blue Alliance Wall
  Pose2d gridPose6 = new Pose2d(new Translation2d(1.613, 4.411), new Rotation2d(Math.toRadians(-179.3)));
  Pose2d gridPose7 = new Pose2d(new Translation2d(1.613, 2.75), new Rotation2d(Math.toRadians(-179.3)));
  Pose2d gridPose8 = new Pose2d(new Translation2d(1.613, 1.08), new Rotation2d(Math.toRadians(-179.3)));

  // Red Alliance Wall
  Pose2d gridPose3 = new Pose2d(new Translation2d(14.92, 4.411), new Rotation2d());
  Pose2d gridPose2 = new Pose2d(new Translation2d(14.92, 2.75), new Rotation2d());
  Pose2d gridPose1 = new Pose2d(new Translation2d(14.92, 1.08), new Rotation2d());

  Pose2d gridPoses[] = { gridPose1, gridPose2, gridPose3, gridPose6, gridPose7, gridPose8 };
  int desiredGrid;

  boolean isBlueAlliance;

  private final CommandMacroPad m_pad;
  private Button desiredNode = Button.button1;

  /** Creates a new TargettingSubsystem. */
  public TargetingSubsystem(CommandMacroPad m_commandMacroPad) {
    m_pad = m_commandMacroPad;
    SmartDashboard.putBoolean("Alliance", false);
  }

  public Pose2d getTargetGridPose() {
    return new Pose2d(gridPoses[desiredGrid].getX(), gridPoses[desiredGrid].getY() + getNodeOffset(),
        new Rotation2d(isBlueAlliance ? 180 : 0));
  }

  public void setGridPose(int inputGrid) {
    desiredGrid = inputGrid;
  }

  public void setNode(int node) {
    if (node >= 1 && node <= 9)
      desiredNode = Button.values()[node - 1];
    else
      return;
  }

  private double getNodeOffset() {
    if(desiredNode == null)
      return 0;

    int allianceReverse = isBlueAlliance ? 1 : -1;
    switch (desiredNode.column) {
      case 1:
        return -POLE_OFFSET * allianceReverse;
      case 2:
        return 0;
      case 3:
        return POLE_OFFSET * allianceReverse;
      default:
        return 0;
    }
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run  
    //isBlueAlliance = SmartDashboard.getBoolean("alliance", true);
    isBlueAlliance = false;
    desiredGrid = selectGrid() + (isBlueAlliance ? 0 : 3) - 1;
    setNode(selectNode().value());
    SmartDashboard.putNumber("grid selected", desiredGrid);
    SmartDashboard.putNumber("node selected", desiredNode.value());
  }



}
