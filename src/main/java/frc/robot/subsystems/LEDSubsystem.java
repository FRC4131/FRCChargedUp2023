package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Code based on the old code from frc2022

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_LED = new AddressableLED(8);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(17);
    int h = 65;
    int s = 255;
    int v = 255;
    private int m_rainbowFirstPixelHue = 0;
    private int lastValue = 0;

    public LEDSubsystem() {
        setHSV(-1, 65, 255, 255);
        m_LED.setLength(m_LEDBuffer.getLength());
        m_LED.setData(m_LEDBuffer);
        m_LED.start();

        SmartDashboard.putNumber("H", 65);
        SmartDashboard.putNumber("S", 255);
        SmartDashboard.putNumber("V", 255);
    }

    @Override
    public void periodic() {
        m_LED.setData(m_LEDBuffer);
        // setHSV(-1, (int)SmartDashboard.getNumber("H", 65),
        // (int)SmartDashboard.getNumber("S", 65),
        // (int)SmartDashboard.getNumber("V", 65));
        // rainbow();
    }

    // If ledIndex is set to -1, then all LEDs will be set to the provided h, s, and
    // v values

    public int getH() {
        return h;
    }

    public int getS() {
        return s;
    }

    public int getV() {
        return v;
    }

    /**
     * Sets HSV values for the specified LED index.
     * <p>
     * <i>Use -1 as the selected index to change all lights
     * 
     * @param ledIndex selected LED
     * @param initH    hue value [0,180)
     * @param initS    saturation value [0,255]
     * @param initV    value value [0, 255]
     */
    public void setHSV(int ledIndex, int h, int s, int v) {
        this.h = h;
        this.s = s;
        this.v = v;
        if (ledIndex == -1) {
            for (int i = 0; i < 17; i++) {
                SmartDashboard.putNumber("LED index", i);
                m_LEDBuffer.setHSV(i, h, s, v);
            }
            return;
        } else {
            m_LEDBuffer.setHSV(ledIndex, h, s, v);
        }
    }

    public void setHSVStupid(int h, int s, int v, boolean workPlease) {
        for (int i = 0; i < 10; i++) {
            m_LEDBuffer.setHSV(i, h, s, v);
        }
        return;
    }

    public int getNumberOfLEDs() {
        return m_LEDBuffer.getLength();
    }

    public void rainbow() {
        for (var i = 0; i < 17; i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / 62)) % 180;
            // Set the value
            m_LEDBuffer.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        m_rainbowFirstPixelHue += 3;
        // Check bounds
        m_rainbowFirstPixelHue %= 180;
    }

    public void pulse(int hue) {
        for (var i = 0; i < 17; i++) {
            final var value = (lastValue + (i * 255 / 17)) % 255;
            m_LEDBuffer.setHSV(i, hue, 255, 180 - value);
        }

        lastValue += 3;
        lastValue %= 255;
    }

}