package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Code based on the old code from frc2022

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED m_LED = new AddressableLED(1);
    private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(62);
    int h;
    int s;
    int v;


    public LEDSubsystem() {
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
    public void setHSV(int ledIndex, int initH, int initS, int initV) {
        h = initH;
        s = initS;
        v = initV;
        if (ledIndex == -1) {
            for (int i = 0; i < m_LEDBuffer.getLength(); ++i) {
                m_LEDBuffer.setHSV(i, h, s, v);
            }
        }
        else {
            m_LEDBuffer.setHSV(ledIndex, h, s, v);
        }
    }

    public int getNumberOfLEDs() {
        return m_LEDBuffer.getLength();
    }
}