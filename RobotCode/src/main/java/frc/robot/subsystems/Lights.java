package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private int colorH = 0;
    private AddressableLED mled;
    private AddressableLEDBuffer mLEDBuffer;
    private Lights() {
        mled = new AddressableLED(Constants.LED_PORT);
        mLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        mled.setLength(mLEDBuffer.getLength());
    }
    private static Lights m_lightsInstance = new Lights();
    private Lights getInstance() {
        return m_lightsInstance;
    }
    @Override
    public void readPeriodicInputs() {
        // TODO Auto-generated method stub
        switch(ColorWheel.getInstance().cDetected()) {
            case 'R': colorH = 0; break;
            case 'Y': colorH = 60; break;
            case 'G': colorH = 120; break;
            case 'B': colorH = 180; break;
            case 'U': break;
        }
        

    }

    @Override
    public void writePeriodicOutputs() {
        // TODO Auto-generated method stub
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            mLEDBuffer.setHSV(i, colorH, 100, 75);
         }
         
         mled.setData(mLEDBuffer);
    }

    @Override
    public void reset() {
        // TODO Auto-generated method stub
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub

    }

}
