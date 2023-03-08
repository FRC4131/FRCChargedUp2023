// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDControlCommand extends CommandBase {

  public enum PATTERN {
    CASCADE, SOLID

  }

  private LEDSubsystem m_ledSubsystem;
  private AddressableLEDBuffer buffer;
  private AddressableLEDBuffer bufferBuffer;

  private Color[] colors;
  private PATTERN pattern;
  private int spacing;
  private double delay;
  private boolean variegated;

  private int pos;
  private boolean runOnce = false;

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
  public LEDControlCommand(LEDSubsystem ledsubsys, double delay, PATTERN pattern, int spacing, boolean vary,
      Color... colors) {

    m_ledSubsystem = ledsubsys;
    buffer = m_ledSubsystem.getBuffer();
    
    bufferBuffer = new AddressableLEDBuffer(buffer.getLength());


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
   * @param vary      whether to add some randomness to the color
   * @param color
   */
  public LEDControlCommand(LEDSubsystem ledsubsys, boolean vary, Color color) {
    this(ledsubsys, 0, PATTERN.SOLID, 1, vary, color);
    runOnce = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    for (int i = 0; i < buffer.getLength(); i++) {
      bufferBuffer.setLED(i, colors[(i / spacing) % colors.length]);
    }

    switch (pattern) {
      case CASCADE:

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

    if (variegated) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, RNGshift(buffer.getLED(i)));
      }
    }

    m_ledSubsystem.update();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Timer.delay(delay);

    switch (pattern) {
      case CASCADE:
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, bufferBuffer.getLED((pos + i) % (buffer.getLength() - 1)));
        }
        break;
      case SOLID:
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setLED(i, colors[pos % colors.length]);
        }

      default:
        break;
    }

    if (variegated) {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setLED(i, RNGshift(buffer.getLED(i)));
      }
    }

    pos = ++pos % buffer.getLength();

    m_ledSubsystem.update();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < buffer.getLength(); i++) {
      buffer.setLED(i, Color.kBlack);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return runOnce;
  }

  private Color RNGshift(Color c) {
/*     var hsv = rgb_to_hsv(c.red, c.green, c.blue);
    hsv[0] += ((int) (Math.random() * 2)) * ((Math.random() * 2 - 1) * 10);
    hsv[0] = Math.min(Math.max(0, hsv[0]), 180);
    return Color.fromHSV(hsv[0], hsv[1], hsv[2]); */
    c = new Color(c.red + ((int) (Math.random() * 2)) * ((Math.random() * 2 - 1) * 25), c.green  + (int) (Math.random() * 2) * ((Math.random() * 2 - 1) * 25), c.blue + (int) (Math.random() * 2) * ((Math.random() * 2 - 1) * 25));
    return c;
  }

  /**
   * 
   * @param r
   * @param g
   * @param b
   * @return a three-long int array of {h, s, v}
   */
  private int[] rgb_to_hsv(double r, double g, double b) {

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
      h = 60 * (((g - b) / diff) % 6);

    // if cmax equal g then compute h
    else if (cmax == g)
      h = (60 * ((b - r) / diff) + 2);

    // if cmax equal b then compute h
    else if (cmax == b)
      h = (60 * ((r - g) / diff) + 4);

    // if cmax equal zero
    if (cmax == 0)
      s = 0;
    else
      s = (diff / cmax);

    // compute v
    double v = cmax;

    return new int[] { (int) h, (int) s * 100, (int) v * 100};
  }
}
