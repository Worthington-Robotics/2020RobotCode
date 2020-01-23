package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import frc.lib.drivers.ColorSensorV3;
import frc.robot.Constants;
import frc.lib.util.Util;

import java.awt.Color;

import static frc.lib.drivers.ColorSensorV3.*;

public class ColorWheel extends Subsystem {

    private TalonSRX colorWheelTalon;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final char[] wheelColors = new char[]{'B', 'Y', 'R', 'G'};
    private String wheelColorsOrder = new String(wheelColors);

    private ColorWheel() {
        colorWheelTalon = new TalonSRX(Constants.COLOR_WHEEL);
        colorSensor = new ColorSensorV3(i2cPort);
        reset();
    }

    private static ColorWheel m_colorWheelInstance = new ColorWheel();

    public static ColorWheel getInstance() {
        return m_colorWheelInstance;
    }

    private ColorWheelIO periodic;

    @Override
    public void readPeriodicInputs() {
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            periodic.fmsColor = colorConvert(gameData.charAt(0));
        } else {
            periodic.fmsColor = 'U';
        }
        periodic.closedLoopError = colorWheelTalon.getClosedLoopError();
    }

    @Override
    public void writePeriodicOutputs() {
        periodic.RGB = new int[]{colorSensor.getRed(), colorSensor.getBlue(), colorSensor.getGreen()};
        if (!periodic.colorMotorPidOn) {
            colorWheelTalon.set(ControlMode.PercentOutput, 0);
        } else {
            colorWheelTalon.set(ControlMode.Position, inchesToTicks(periodic.distance));
        }
    }

    private double inchesToTicks(double inches) {
        return Constants.ENCODER_5046_CPR / (Constants.COLOR_WHEEL_SPINNER_DIA * Math.PI);
    }

    @Override
    public void outputTelemetry() {

    }

    /**
     * Returns the color that the sensor sees on the wheel
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

        if(hsv[2] < Constants.COLOR_WHEEL_VAL_LIMIT || hsv[1] < Constants.COLOR_WHEEL_SAT_LIMIT) {
            return 'U';
        }
        else {
            if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_RED_HUE1, Constants.COLOR_WHEEL_HUE_ERROR) || Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_RED_HUE2, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'R';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_YELLOW_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'Y';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_GREEN_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'G';
            }
            else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_BLUE_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
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
    private static char colorConvert(char color) {
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
     * Picks the direction the wheel has to spin for maximum efficiency and calculates distance to spin the control panel
     *
     */
    private void distance() {
        periodic.colorDirectionCalc = wheelColorsOrder.indexOf(colorConvert(periodic.fmsColor)) - wheelColorsOrder.indexOf(cDetected());
        if (wheelColorsOrder.indexOf(colorConvert(periodic.fmsColor)) == -1 || wheelColorsOrder.indexOf(cDetected()) == -1) {
            periodic.distance = 0;
        } else if (periodic.colorDirectionCalc == -3) {
            periodic.distance = -12.5;
        }
        else if (periodic.colorDirectionCalc == 3) {
            periodic.distance = 12.5;
        } else {
            periodic.distance = periodic.colorDirectionCalc * 12.5;
        }
    }

    @Override
    public void reset() {
        periodic = new ColorWheelIO();
        colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes18bit, ColorSensorMeasurementRate.kColorRate100ms, GainFactor.kGain1x);
        configTalon();
    }
    public void setDemand(double newDemand) {
        periodic.demand = newDemand;
    }

    public boolean isOnTarget() {
        return Util.epsilonEquals(periodic.closedLoopError, 0, 20);
    }

    public boolean checkColor() {
        return periodic.fmsColor == cDetected();
    }

    public void setColorMotorPidOn(boolean motorOn) {
        periodic.colorMotorPidOn = motorOn;
    }

    private void configTalon() {
        colorWheelTalon.setSensorPhase(true);
        colorWheelTalon.selectProfileSlot(0, 0);
        colorWheelTalon.config_kF(0, Constants.COLOR_WHEEL_KF);
        colorWheelTalon.config_kP(0, Constants.COLOR_WHEEL_KP);
        colorWheelTalon.config_kI(0, Constants.COLOR_WHEEL_KI);
        colorWheelTalon.config_kD(0, Constants.COLOR_WHEEL_KD);
        colorWheelTalon.config_IntegralZone(0, 300);
        colorWheelTalon.setInverted(false);
        colorWheelTalon.setNeutralMode(NeutralMode.Brake);
        colorWheelTalon.configVoltageCompSaturation(Constants.COLOR_WHEEL_VCOMP);
        colorWheelTalon.enableVoltageCompensation(true);
    }

    public class ColorWheelIO extends Subsystem.PeriodicIO {
        public double closedLoopError = 0;
        public char fmsColor = 'U';
        public double distance = 0;
        public int[] RGB = new int[]{0, 0, 0};
        public int colorDirectionCalc;
        public double demand = 0.0;
        public boolean colorMotorPidOn = false;
    }
}
