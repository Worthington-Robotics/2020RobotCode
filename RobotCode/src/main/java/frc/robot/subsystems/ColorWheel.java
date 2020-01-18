package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import frc.lib.util.Util;

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

    public char colorFromRGB(int[] RGB){
        //RGB Values: Blue: 0, 255, 255. Green: 0, 255, 0. Red: 255, 0, 0. Yellow: 255, 255, 0.
        //H Values: Blue: 180. Green: 120. Yellow: 60. Red: 0
        final int redH1 = 0;
        final int redH2 = 360;
        final int yellowH = 60;
        final int greenH = 120;
        final int blueH = 180;
        final int error = 25;
        int h = RGBtoH(RGB);
        if (Util.epsilonEquals(redH1, error) || Util.epsilonEquals(redH2, error)) {
            return 'R';
        } else if (Util.epsilonEquals(yellowH, error)) {
            return 'Y';
        } else if (Util.epsilonEquals(greenH, error)) {
            return 'G';
        } else if (Util.epsilonEquals(blueH, error)) {
            return 'B';
        } else {
            return 'U';
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

    /**
     * Coverts RGB values to a Hue value
     *
     * @param rgb RGB in an ArrayList
     * @return Hue value of RGB
     */
    private static int RGBtoH(int[] rgb) {

        int h, min, max;
        double delta;

        min = Math.min(Math.min(rgb[0], rgb[1]), rgb[2]);
        max = Math.max(Math.max(rgb[0], rgb[1]), rgb[2]);

        delta = max - min;

        //
        if (max == 0) {
            h = -1;
            return h;
        }

        // H
        if (rgb[0] == max) {
            h = (int) (((rgb[1] - rgb[2]) / delta) % 6); // between yellow & magenta
        } else if (rgb[1] == max) {
            h = (int) (2 + (rgb[2] - rgb[0]) / delta); // between cyan & yellow
        } else {
            h = (int) (4 + (rgb[0] - rgb[1]) / delta); // between magenta & cyan
        }

        h *= 60;    // degrees

        if (h < 0) {
            h += 360;
        }

        return h;
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
