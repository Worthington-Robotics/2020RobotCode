package frc.robot.subsystems;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.lib.util.Util;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;

import java.awt.event.MouseWheelListener;

public class ColorPicker extends Subsystem{

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private int[] RGB = new int[]{0, 0, 0};

    private ColorPicker() {

    }

    private static ColorPicker m_colorPickerInstance = new ColorPicker();

    public static ColorPicker getInstance() {
        return m_colorPickerInstance;
    }

    @Override
    public void readPeriodicInputs() {
        Color detectedColor = colorSensor.getColor();
    }

    @Override
    public void writePeriodicOutputs() {
        RGB[0] = colorSensor.getRed();
        RGB[1] = colorSensor.getBlue();
        RGB[2] = colorSensor.getGreen();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Vision/Color Detected", colorCovert(cDetected()).toString());
    }

    private WheelColor cDetected() {
        //RGB Values: Blue: 0, 255, 255. Green: 0, 255, 0. Red: 255, 0, 0. Yellow: 255, 255, 0.//
        int error = 10;
        if (Util.epsilonEquals(RGB[0], 0, error)) {
            if (Util.epsilonEquals(RGB[3], 0, error)) {
                return WheelColor.GREEN;
            }
            else if (Util.epsilonEquals(RGB[3], 255, error)) {
                return WheelColor.BLUE;
            }
            else {
                return WheelColor.NOT_A_COLOR;
            }
        }
        else if (Util.epsilonEquals(RGB[1], 0, error)) {
            return WheelColor.RED;
        }
        else if (Util.epsilonEquals(RGB[1], 255, error)) {
            return WheelColor.YELLOW;
        }
        return WheelColor.NOT_A_COLOR;
    }

    private WheelColor colorCovert(WheelColor color) {
        if(color == WheelColor.BLUE) {
            return WheelColor.RED;
        }
        else if(color == WheelColor.RED) {
            return WheelColor.BLUE;
        }
        else if(color == WheelColor.YELLOW) {
            return WheelColor.GREEN;
        }
        else if(color == WheelColor.GREEN) {
            return WheelColor.YELLOW;
        }
        else {
            return WheelColor.NOT_A_COLOR;
        }
    }

    @Override
    public void reset() {

    }

    enum WheelColor {
        RED,
        BLUE,
        GREEN,
        YELLOW,
        NOT_A_COLOR;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}
