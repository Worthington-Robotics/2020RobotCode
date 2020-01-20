package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import frc.robot.Constants;
import frc.lib.util.Util;
import java.awt.Color;

public class ColorWheel extends Subsystem {

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final char[] wheelColors = new char[]{'B', 'Y', 'R', 'G'};

    private ColorWheel() {
        periodic = new PeriodicIO();
        colorSensor = new ColorSensorV3(i2cPort);
    }

    private static ColorWheel m_colorWheelInstance = new ColorWheel();

    public static ColorWheel getInstance() {
        return m_colorWheelInstance;
    }

    private PeriodicIO periodic;

    @Override
    public void readPeriodicInputs() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            periodic.fmsColor = colorCovert(gameData.charAt(0));
        } else {
            periodic.fmsColor = 'U';
        }
    }

    @Override
    public void writePeriodicOutputs() {
        periodic.RGB = new int[]{colorSensor.getRed(), colorSensor.getBlue(), colorSensor.getGreen()};
    }

    @Override
    public void outputTelemetry() {

    }

    /**
     * This returns the color that the sensor sees on the wheel
     *
     * @return color the sensor sees on the wheel (Red, Yellow, Green, or Blue)
     */
    public char cDetected() {
        return colorFromRGB(periodic.RGB);
    }

    public static char colorFromRGB(int[] RGB){
        //RGB Values: Blue: 0, 255, 255. Green: 0, 255, 0. Red: 255, 0, 0. Yellow: 255, 255, 0.
        //H Values: Blue: 180. Green: 120. Yellow: 60. Red: 0

        float[] hsv = new float[3];
        Color.RGBtoHSB(RGB[0], RGB[1], RGB[2], hsv);
        //System.out.println(hsv[0] + ", " + hsv[1] + ", " + hsv[2]);
        hsv[0] *= 360;
        hsv[1] *= 100;
        hsv[2] *= 100;
        //System.out.println(hsv[0] + ", " + hsv[1] + ", " + hsv[2]);

        if(hsv[2] < Constants.valLimit || hsv[1] < Constants.satLimit) {
            return 'Q';
        }
        else {
            if (Util.epsilonEquals(hsv[0], Constants.redH1, Constants.error) || Util.epsilonEquals(hsv[0], Constants.redH2, Constants.error)) {
                return 'R';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.yellowH, Constants.error)) {
                return 'Y';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.greenH, Constants.error)) {
                return 'G';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.blueH, Constants.error)) {
                return 'B';
            } else {
                return 'U';
            }
        }
    }

    /**
     * Takes colors and converts it from what the robot sees
     * to what the field sees or visa versa
     *
     * @param color Color Red, Yellow, Green, Blue that the robot is seeing
     *              or what the field is seeing.
     * @return Returns the color that the field sensor is sees
     * or what the robot sees
     */
    private static char colorCovert(char color) {
        switch (color) {
            case 'R':
                return 'B';
            case 'B':
                return 'R';
            case 'Y':
                return 'G';
            case 'G':
                return 'Y';
            default:
                return 'U';
        }
    }

    /**
     * Picks the direction the wheel has to spin for maximum efficiency
     *
     * @param color takes in a character 'R', 'Y', 'G', 'B' of what the robot sees
     */
    private void direction(char color) {
        String wheelColorsOrder = new String(wheelColors);
        if (wheelColorsOrder.indexOf(colorCovert(periodic.fmsColor)) - wheelColorsOrder.indexOf(cDetected()) < -1 || wheelColorsOrder.indexOf(colorCovert(periodic.fmsColor)) - wheelColorsOrder.indexOf(cDetected()) == 1) {
            periodic.direction = 0;
        } else {
            periodic.direction = 1;
        }
    }

    @Override
    public void reset() {

    }

    public class PeriodicIO extends Subsystem.PeriodicIO {
        char fmsColor = 'U';
        int direction = 1; //1 for right, 0 for left
        int[] RGB = new int[]{0, 0, 0};
    }


}
