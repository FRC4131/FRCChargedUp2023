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
  ShuffleboardTab tab = Shuffleboard.getTab("tab");
  GenericEntry entry = tab.add("setAlliance", "blue").getEntry();

  private final CommandMacroPad m_pad;

  /** Creates a new TargettingSubsystem. */
  public TargetingSubsystem(CommandMacroPad m_commandMacroPad) {
    m_pad = m_commandMacroPad;

  }

  public Pose2d getTargetGridPose() {
    return gridPoses[desiredGrid];
  }

  public void setGridPose(int inputGrid) {
    desiredGrid = inputGrid;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    isBlueAlliance = entry.getString("blue").equalsIgnoreCase("blue") ? true : false;
    SmartDashboard.putBoolean("alliance", isBlueAlliance);
    desiredGrid = divineSelectedButtonPertainingToGridSelection() * (isBlueAlliance ? 1 : 2) - 1;
    divineSelectedButtonPertainingToNodeSelection();
  }

  private int divineSelectedButtonPertainingToGridSelection() {
    var butons = m_pad.getButtonsPressed();
    for (Button button : butons) {
      if (button.row == 4) {
        SmartDashboard.putNumber("grid selected", button.column);
        return button.column;
      }
    }
    // only returns from here if no grid selected
    // 0 as default is probably fine
    return 0;
  }

  private int divineSelectedButtonPertainingToNodeSelection() {
    var butons = m_pad.getButtonsPressed();
    for (Button button : butons) {
      if (button.row < 4) {
        SmartDashboard.putNumber("node selected", button.value());
        return button.value();
      }
    }
    // only returns from here if no grid selected
    // 0 as default is probably fine
    return 0;
  }

  public void swapAlliance() {
    File f = new File("D:\\code.py");
    Scanner fileIn;
    try {
      fileIn = new Scanner(f);
      String in = "";
      for (int i = 0; i < 11; i++) {
        in += fileIn.nextLine();
      }
      var specialLine = fileIn.nextLine();
      if (specialLine.toCharArray()[0] == '#') {
        specialLine = specialLine.substring(1, specialLine.length());
      } else {
        specialLine = "#" + specialLine;
      }
      in += specialLine;
      while (fileIn.hasNextLine()) {
        in += fileIn.nextLine();
      }
      PrintStream ps;

      ps = new PrintStream("D:\\code.py");

      ps.print(in);

      ps.close();
      fileIn.close();
    } catch (FileNotFoundException e1) {
      
    }
  }
}
