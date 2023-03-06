package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Code based on the old code from frc2022

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_LED = new AddressableLED(0);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(62);
    int h = 130;
    int s = 255;
    int v = 255;


    public LEDSubsystem() {
        m_LEDBuffer.setHSV(0, 130, 255, 255);
        m_LED.setLength(m_LEDBuffer.getLength());
        m_LED.setData(m_LEDBuffer);
        m_LED.start();
    }

    @Override
    public void periodic() {
        m_LED.setData(m_LEDBuffer);
    }

    // If ledIndex is set to -1, then all LEDs will be set to the provided h, s, and v values

    public int getH()
    {
        return h;
    }

    public int getS()
    {
        return s;
    }

    public int getV()
    {
        return v;
    }
    
    /**
     * Sets HSV values for the specified LED index.
     * <p><i>Use -1 as the selected index to change all lights
     * @param ledIndex selected LED
     * @param initH hue value [0,180)
     * @param initS saturation value [0,255]
     * @param initV value value [0, 255]
     */
    public void setHSV(int ledIndex, int h, int s, int v) {
        if (ledIndex == -1) {
            for (int i = 0; i < 62; i++) {
                SmartDashboard.putNumber("LED index", i);
                m_LEDBuffer.setHSV(i, h, s, v);
            }
            return;
        }
        else {
            m_LEDBuffer.setHSV(ledIndex, h, s, v);
        }
    }

    public int getNumberOfLEDs() {
        return m_LEDBuffer.getLength();
    }
}