package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.ColorSensorV3;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import frc.robot.Constants;
import frc.lib.util.Util;
import edu.wpi.first.wpilibj.util.Color;
import static frc.lib.drivers.ColorSensorV3.*;

public class ColorWheel extends Subsystem {

    private final TalonSRX colorWheelTalon;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final char[] wheelColors = new char[] { 'B', 'Y', 'R', 'G' };
    private final String wheelColorsOrder = new String(wheelColors);
    // Color Match Stuff
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final Color kBlueTarget = ColorMatch.makeColor(0.128, 0.413, 0.459);
    private final Color kGreenTarget = ColorMatch.makeColor(0.172, 0.564, 0.264);
    private final Color kRedTarget = ColorMatch.makeColor(0.498, 0.352, 0.150);
    private final Color kYellowTarget = ColorMatch.makeColor(0.315, 0.553, 0.132);

    private ColorWheel() {
        colorWheelTalon = new TalonSRX(Constants.COLOR_WHEEL);
        colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
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
            periodic.fms_color = colorConvert(gameData.charAt(0));
        } else {
            periodic.fms_color = 'U';
        }
        periodic.close_loop_error = colorWheelTalon.getClosedLoopError();
        periodic.detected_color = colorSensor.getColor();
        periodic.RGB = new double[] { periodic.detected_color.red, periodic.detected_color.blue,
                periodic.detected_color.green };
        periodic.color_sensed = colorMatch();
    }

    @Override
    public void writePeriodicOutputs() {
        if (!periodic.color_motor_pid_on) {
            colorWheelTalon.set(ControlMode.Position, inchesToTicks(Constants.COLOR_WHEEL_ROTATION_DISTANCE));
        } else {
            colorWheelTalon.set(ControlMode.Position, inchesToTicks(periodic.distance));
        }
    }

    private double inchesToTicks(final double inches) {
        return (inches / (Constants.COLOR_WHEEL_SPINNER_DIA * Math.PI)) / Constants.ENCODER_5046_CPR;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Color Wheel/Red", periodic.detected_color.red);
        SmartDashboard.putNumber("Color Wheel/Blue", periodic.detected_color.blue);
        SmartDashboard.putNumber("Color Wheel/Green", periodic.detected_color.green);
        SmartDashboard.putString("Color Wheel/Detected Color", periodic.color_sensed);
        SmartDashboard.putNumber("Confidence", m_colorMatcher.matchClosestColor(periodic.detected_color).confidence);
    }

    /**
     * Returns the color that the sensor sees on the wheel
     *
     * @return color the sensor sees on the wheel (Red, Yellow, Green, or Blue)
     */
    public char cDetected() {
        return colorFromRGB(periodic.RGB);
    }

    public static char colorFromRGB(final double[] RGB) {
        // RGB Values: Blue: 0, 255, 255. Green: 0, 255, 0. Red: 255, 0, 0. Yellow: 255,
        // 255, 0.
        // H Values: Blue: 180. Green: 120. Yellow: 60. Red: 0

        final float[] hsv = new float[3];
        java.awt.Color.RGBtoHSB((int) (RGB[0] * 255), (int) (RGB[1] * 255), (int) (RGB[2] * 255), hsv);
        // System.out.println(hsv[0] + ", " + hsv[1] + ", " + hsv[2]);
        hsv[0] *= 360;
        hsv[1] *= 100;
        hsv[2] *= 100;
        // System.out.println(hsv[0] + ", " + hsv[1] + ", " + hsv[2]);

        if (hsv[2] < Constants.COLOR_WHEEL_VAL_LIMIT || hsv[1] < Constants.COLOR_WHEEL_SAT_LIMIT) {
            return 'U';
        } else {
            if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_RED_HUE1, Constants.COLOR_WHEEL_HUE_ERROR)
                    || Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_RED_HUE2, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'R';
            } else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_YELLOW_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'Y';
            } else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_GREEN_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'G';
            } else if (Util.epsilonEquals(hsv[0], Constants.COLOR_WHEEL_BLUE_HUE, Constants.COLOR_WHEEL_HUE_ERROR)) {
                return 'B';
            } else {
                return 'U';
            }
        }
    }

    /**
     * Takes colors and converts it from what the robot sees to what the field sees
     * or visa versa
     *
     * @param color Color Red, Yellow, Green, Blue that the robot is seeing or what
     *              the field is seeing.
     * @return Returns the color that the field sensor is sees or what the robot
     *         sees
     */
    private static char colorConvert(final char color) {
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
     * Picks the direction the wheel has to spin for maximum efficiency and
     * calculates distance to spin the control panel
     *
     */
    private void distance() {
        periodic.color_direction_calc = wheelColorsOrder.indexOf(colorConvert(periodic.fms_color))
                - wheelColorsOrder.indexOf(cDetected());
        if (wheelColorsOrder.indexOf(colorConvert(periodic.fms_color)) == -1
                || wheelColorsOrder.indexOf(cDetected()) == -1) {
            periodic.distance = 0;
        } else if (periodic.color_direction_calc == -3) {
            periodic.distance = -12.5;
        } else if (periodic.color_direction_calc == 3) {
            periodic.distance = 12.5;
        } else {
            periodic.distance = periodic.color_direction_calc * 12.5;
        }
    }

    @Override
    public void reset() {
        periodic = new ColorWheelIO();
        colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes16bit,
                ColorSensorMeasurementRate.kColorRate50ms, GainFactor.kGain3x);
        configTalon();
    }

    public void setDemand(final double newDemand) {
        periodic.demand = newDemand;
    }

    public boolean isOnTarget() {
        return Util.epsilonEquals(periodic.close_loop_error, 0, 20);
    }

    public boolean checkColor() {
        return periodic.fms_color == cDetected();
    }

    public void setColorMotorPidOn(final boolean motorOn) {
        periodic.color_motor_pid_on = motorOn;
    }

    public String colorMatch() {

        /**
         * Run the color match algorithm on our detected color
         */
        String colorString;
        ColorMatchResult match = m_colorMatcher.matchClosestColor(periodic.detected_color);

        if(match.confidence <= .95)
            return "Unknown";

        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }

        return colorString;
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

    public LogData getLogger(){
        return periodic;
    }

    public class ColorWheelIO extends Subsystem.PeriodicIO {
        public double close_loop_error = 0;
        public char fms_color = 'U';
        public double distance = 0;
        public double[] RGB = new double[] { 0, 0, 0 };
        public int color_direction_calc;
        public double demand = 0.0;
        public boolean color_motor_pid_on = false;
        public Color detected_color = Color.kBlack;
        public String color_sensed = "Unknown";
    }
}
