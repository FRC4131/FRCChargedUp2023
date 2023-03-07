// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;
import java.util.regex.Pattern;

import com.ctre.phoenix.led.ColorFlowAnimation;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDControlCommand extends CommandBase {

  enum PATTERN {
    CHASING, SOLID

  }

  private LEDSubsystem m_ledSubsystem;
  private AddressableLEDBuffer buffer;
  private AddressableLEDBuffer bufferBuffer;

  private Color[] colors;
  private PATTERN pattern;
  private int spacing;
  private float delay;
  private boolean variegated;

  private int pos;

  /**
   * A command that provides for the creation of a customizable led subsystem
   * 
   * @param ledsubsys
   * @param delay     time between each lighting update
   * @param pattern
   * @param spacing   number of consecutive lights to use the same color
   * @param vary      whether to add some randomness to the colors
   * @param colors    ordered list of colors
   */
  public LEDControlCommand(LEDSubsystem ledsubsys, float delay, PATTERN pattern, int spacing, boolean vary, Color... colors) {

    m_ledSubsystem = ledsubsys;
    buffer = m_ledSubsystem.getBuffer();
    bufferBuffer = new AddressableLEDBuffer(colors.length * spacing);

    this.colors = colors;
    this.pattern = pattern;
    this.spacing = spacing;
    this.delay = delay;
    this.variegated = vary;

    addRequirements(ledsubsys);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /**
   * Creates a command that sets LED strip to single sold color
   * 
   * @param ledsubsys
   * @param vary whether to add some randomness to the color
   * @param color
   */
  public LEDControlCommand(LEDSubsystem ledsubsys, boolean vary, Color color) {
    this(ledsubsys, 99999, PATTERN.SOLID, 1, vary, color);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    switch (pattern) {
      case CHASING:

        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, colors[(i / spacing) % colors.length]);
        }

        break;

      case SOLID:

        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, colors[0]);
        }

        break;

      default:
        break;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Timer.delay(delay);

    switch (pattern) {
      case CHASING:
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, bufferBuffer.getLED((pos + i) % buffer.getLength()));
        }
        break;
      case SOLID:
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, colors[pos % colors.length]);
        }

      default:
        break;
    }

    if(variegated){
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, RNGshift(buffer.getLED(i)));
      }
    }

    pos = ++pos % buffer.getLength();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, new Color(0, 0, 0));
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private Color RNGshift(Color c){
    var hsv = rgb_to_hsv(c.red, c.green, c.blue);
    hsv[0] += (int)(Math.random() * 2) * ((Math.random() * 2 - 1) * 8);
    return Color.fromHSV(hsv[0], hsv[1], hsv[2]);
  }


  private int[] rgb_to_hsv(double r, double g, double b)
    {
  
        // R, G, B values are divided by 255
        // to change the range from 0..255 to 0..1
        r = r / 255.0;
        g = g / 255.0;
        b = b / 255.0;
  
        // h, s, v = hue, saturation, value
        double cmax = Math.max(r, Math.max(g, b)); // maximum of r, g, b
        double cmin = Math.min(r, Math.min(g, b)); // minimum of r, g, b
        double diff = cmax - cmin; // diff of cmax and cmin.
        double h = -1, s = -1;
          
        // if cmax and cmax are equal then h = 0
        if (cmax == cmin)
            h = 0;
  
        // if cmax equal r then compute h
        else if (cmax == r)
            h = (60 * ((g - b) / diff) + 360) % 360;
  
        // if cmax equal g then compute h
        else if (cmax == g)
            h = (60 * ((b - r) / diff) + 120) % 360;
  
        // if cmax equal b then compute h
        else if (cmax == b)
            h = (60 * ((r - g) / diff) + 240) % 360;
  
        // if cmax equal zero
        if (cmax == 0)
            s = 0;
        else
            s = (diff / cmax) * 100;
  
        // compute v
        double v = cmax * 100;
        
        return new int[]{(int) h,(int)s,(int)v};
    }
}
