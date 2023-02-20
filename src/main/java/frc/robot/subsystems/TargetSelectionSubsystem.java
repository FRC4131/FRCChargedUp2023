// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetSelectionSubsystem extends SubsystemBase {
  /** Creates a new TargetSelectionSubsystem. */
  public TargetSelectionSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  
}

/*
 we have an array of locations for each target within a grid
 activated by pressing 'score' button after selecting position to score
 poisition is either (x,y) converted to encoders or just encoded values
 
 */
