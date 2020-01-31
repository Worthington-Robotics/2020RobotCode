package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class Lights extends Subsystem {

    private static Lights m_lightsInstance = new Lights();

    public static Lights getInstance() {
        return m_lightsInstance;
    }

    private int colorH = 0;
    private AddressableLED mled;
    private AddressableLEDBuffer mLEDBuffer;
    private int numberOfBalls;
    private boolean targeted, uprightsUp;
    private Color allianceColor;
    private lightModes currentLightMode = lightModes.indexNum;
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
        if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
            allianceColor = Color.kBlue;
        } else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
            allianceColor = Color.kRed;
        } else {
            allianceColor = Color.kChocolate;
        }
        colorH = interpretColor(ColorWheel.getInstance().cDetected());
        //TODO Add Light Implementation for indexer
        //TODO Implement targeting
    }

    @Override
    public void writePeriodicOutputs() {
        switch (currentLightMode) {
            case colorWheel: for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                mLEDBuffer.setHSV(i, colorH, 100, 75);
            } break;
            case targeting: if (targeted){
                for(var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setRGB(i, 13, 239, 66);
                    }
            } else {
                for(var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setRGB(i, 220, 61, 42);
                    }
            } break;
            case indexNum: if (numberOfBalls >= 1 && numberOfBalls <= 4) {
                    for(var i = 0; i < (mLEDBuffer.getLength() * (.2*numberOfBalls)); i++) {
                    mLEDBuffer.setRGB(i, 254, 226, 62);
                    }
            } break;
            default: for(var i = 0; i < mLEDBuffer.getLength(); i++) {
                mLEDBuffer.setLED(i, allianceColor);
            }break;
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
    
    enum lightModes {
        targeting,
        colorWheel,
        indexNum;
    }
    
}
