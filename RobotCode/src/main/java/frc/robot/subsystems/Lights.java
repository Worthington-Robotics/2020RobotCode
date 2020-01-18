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
    public static Lights getInstance() {
        return m_lightsInstance;
    }

    @Override
    public void readPeriodicInputs() {
        interpretColor(ColorWheel.getInstance().cDetected());
    }

    @Override
    public void writePeriodicOutputs() {
        for (var i = 0; i < mLEDBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            mLEDBuffer.setHSV(i, colorH, 100, 75);
         }
         
         mled.setData(mLEDBuffer);
    }
    
    /*
    * This converts the char inputted into a hue value.
    * 
    * @param color color RGBY as a char, capitals only
    * @return hue value of the color inputted
    */
    public int interpretColor(char color) {
        switch(color) {
            case 'R': return 0; 
            case 'Y': return 60; 
            case 'G': return 120;
            case 'B': return 180;
            default: return 300;
            
        }
    }

    @Override
    public void reset() {
    }

    @Override
    public void outputTelemetry() {

    }
    
}
