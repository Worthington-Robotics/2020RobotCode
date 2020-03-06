package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.statemachine.StateMachine;
import frc.robot.Constants;

public class Lights extends Subsystem {
    private AddressableLED mled;
    private AddressableLEDBuffer mLEDBuffer;
    private int numberOfBalls;
    private boolean targeted, uprightsUp, intakeDown, climbUp, FMSOn;
    private Color allianceColor, colorWheelColor;
    private int state = 0;
    private lightModes currentLightMode = lightModes.beforeStart;
    private Value intakeState;

    private Lights() {
        mled = new AddressableLED(Constants.LED_PORT);
        mLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        reset();
    }

    private static Lights m_lightsInstance = new Lights();

    public static Lights getInstance() {
        return m_lightsInstance;
    }

    @Override
    public void readPeriodicInputs() {
        // intakeState = Superstructure.getInstance().intakeDown();
        FMSOn = DriverStation.getInstance().isFMSAttached();
        if (intakeState == Value.kForward) {
            intakeDown = true;
        } else {
            intakeDown = false;
        }
        if (FMSOn = true) {
            currentLightMode = lightModes.allianceColor;
            if (DriverStation.getInstance().getAlliance() == Alliance.Blue) {
                allianceColor = Color.kFirstBlue;
            } else if (DriverStation.getInstance().getAlliance() == Alliance.Red) {
                allianceColor = Color.kDarkRed;
            } else {
                allianceColor = Color.kChocolate;
            }
        } else {
            currentLightMode = lightModes.beforeStart;
        }
        // TODO Implement targeting
        uprightsUp = Climber.getInstance().getUnfolded();
        climbUp = Climber.getInstance().getClimbed();
        /*if (uprightsUp && !climbUp) {
            currentLightMode = lightModes.colorWheel;
            switch (ColorWheel.getInstance().cDetected()) {
            case 'U':
                colorWheelColor = Color.kBlack;
                break;
            case 'R':
                colorWheelColor = Color.kRed;
                break;
            case 'G':
                colorWheelColor = Color.kDarkGreen;
                break;
            case 'Y':
                colorWheelColor = Color.kGold;
                break;
            case 'B':
                colorWheelColor = Color.kBlue;
                break;
            }
        } else {*/
            currentLightMode = lightModes.targeting;
        //}
        // Testing
        // currentLightMode = lightModes.Testing;
        // colorWheelColor = Color.kBlue;
        // numberOfBalls = 5;
    }

    @Override
    public void writePeriodicOutputs() {
        switch (currentLightMode) {
        case Testing:
            for (var i = 0; i < (int)(mLEDBuffer.getLength() * (state / 7)); i++) {
            mLEDBuffer.setLED(i, Color.kBlue);
        }
        case colorWheel:
            for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                // Sets the specified LED to the RGB values for red
                mLEDBuffer.setLED(i, colorWheelColor);
                // System.out.println("Lights Set");
            }
            break;
        case targeting:
            if (Shooter.getInstance().onTarget()) {
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setRGB(i, 0, 150, 0);
                }
            } else {
                for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                    mLEDBuffer.setRGB(i, 150, 0, 0);
                }
            }
            // System.out.println("Lights Set");
            break;
        case indexNum:
            if (numberOfBalls >= 1 && numberOfBalls <= 4) {
                for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                    mLEDBuffer.setLED(i, Color.kYellow);
                }
            } else {
                for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                    mLEDBuffer.setHSV(i, i * (239 / mLEDBuffer.getLength()), 226, 62);
                }
            }
            // System.out.println("Lights Set");
            break;
        case allianceColor:
            for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                mLEDBuffer.setLED(i, allianceColor);
            }
            // System.out.println("Lights Set");
            break;
        case beforeStart:
            for (var i = 0; i < (mLEDBuffer.getLength() * (.2 * numberOfBalls)); i++) {
                mLEDBuffer.setHSV(i, i * ((239 / mLEDBuffer.getLength())), 226, 62);
            }
            break;
        default:
            for (var i = 0; i < mLEDBuffer.getLength(); i++) {
                mLEDBuffer.setLED(i, Color.kPurple);
            }
            // System.out.println("Lights Set");
            break;
        }
        mled.setData(mLEDBuffer);
    }

    /*
     * This converts the char inputted into a hue value.
     * 
     * @param color color RGBY as a char, capitals only
     * 
     * @return hue value of the color inputted
     */
    public int interpretColor(char color) {
        switch (color) {
        case 'R':
            return 0;
        case 'Y':
            return 60;
        case 'G':
            return 120;
        case 'B':
            return 180;
        default:
            return 300;

        }
    }

    public void testLights(int state)
    {
        this.state = state;
        currentLightMode = lightModes.Testing;
    }

    @Override
    public void reset() {
        mLEDBuffer = new AddressableLEDBuffer(60);
        mled.setLength(mLEDBuffer.getLength());
        mled.setData(mLEDBuffer);
        mled.start();
        currentLightMode = lightModes.allianceColor;

    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Length of String", mLEDBuffer.getLength());
    }

    enum lightModes {
        targeting, colorWheel, indexNum, allianceColor, beforeStart, Testing;
    }

}
