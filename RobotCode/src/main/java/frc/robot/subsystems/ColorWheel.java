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
import frc.lib.drivers.ColorSensorV3.*;
import frc.lib.util.Util;
import edu.wpi.first.wpilibj.util.Color;

public class ColorWheel extends Subsystem {

    private final TalonSRX colorWheelTalon;
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final char[] wheelColors = new char[] { 'B', 'Y', 'R', 'G' };
    private final String wheelColorsOrder = new String(wheelColors);
    // Color Match Stuff
    private final ColorMatch m_colorMatcher = new ColorMatch();

    private ColorWheel() {
        colorWheelTalon = new TalonSRX(Constants.COLOR_WHEEL);
        colorSensor = new ColorSensorV3(i2cPort);
        m_colorMatcher.addColorMatch(Constants.kBlueTarget);
        m_colorMatcher.addColorMatch(Constants.kGreenTarget);
        m_colorMatcher.addColorMatch(Constants.kRedTarget);
        m_colorMatcher.addColorMatch(Constants.kYellowTarget);
        reset();
    }

    private static ColorWheel m_colorWheelInstance = new ColorWheel();

    public static ColorWheel getInstance() {
        return m_colorWheelInstance;
    }

    private ColorWheelIO periodic;

    @Override
    public void readPeriodicInputs() {
        periodic.demand = Constants.MASTER.getRawAxis(3);
        String gameData;
        gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            periodic.fms_color = colorConvert(gameData.charAt(0));
        } else {
            periodic.fms_color = 'U';
        }
        periodic.detected_color = colorSensor.getColor();
        periodic.RGB = new double[] { periodic.detected_color.red, periodic.detected_color.blue,
                periodic.detected_color.green };
        periodic.color_sensed = colorMatch();
        periodic.close_loop_error = colorWheelTalon.getClosedLoopError();
        if (!(periodic.color_sensed.charAt(0) == 'U')) {
            periodic.color_wheel_reading = true;
        } else {
            periodic.color_wheel_reading = false;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (periodic.isEnabled) {
            if (periodic.CCW)
                colorWheelTalon.set(ControlMode.PercentOutput, -.2);
            else {
                colorWheelTalon.set(ControlMode.PercentOutput, .2);
            }
        }
        else
        {
            colorWheelTalon.set(ControlMode.PercentOutput, 0);
        }

        /*
         * periodic.RGB = new double[] { (periodic.detected_color.red * 255),
         * (periodic.detected_color.green * 255), (periodic.detected_color.blue * 255)
         * }; if (periodic.color_wheel_reading) { if (periodic.fms_color == 'U') { if
         * (!periodic.color_motor_pid_on) { colorWheelTalon.set(ControlMode.Position,
         * inchesToTicks(Constants.COLOR_WHEEL_ROTATION_DISTANCE)); } } else { if
         * (!periodic.color_motor_pid_on) { checkIfDone();
         * colorWheelTalon.set(ControlMode.Position, inchesToTicks(periodic.demand)); }
         * } } else { }
         */
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Color Wheel/Power", periodic.demand);
        SmartDashboard.putNumber("Color Wheel/Red", periodic.detected_color.red);
        SmartDashboard.putNumber("Color Wheel/Blue", periodic.detected_color.blue);
        SmartDashboard.putNumber("Color Wheel/Green", periodic.detected_color.green);
        SmartDashboard.putString("Color Wheel/Detected Color", periodic.color_sensed);
        SmartDashboard.putString("Color Wheel/FMS Color", periodic.fms_color + "");
        SmartDashboard.putNumber("Color Wheel/Confidence", m_colorMatcher.matchClosestColor(periodic.detected_color).confidence);
    }

    // User Created Methods

    private double inchesToTicks(double inches) {
        return (inches / (Constants.COLOR_WHEEL_SPINNER_DIA * Math.PI)) / Constants.COLOR_ENCODER_CPR;
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
                - wheelColorsOrder.indexOf(periodic.color_sensed.charAt(0));
        if (wheelColorsOrder.indexOf(colorConvert(periodic.fms_color)) == -1
                || wheelColorsOrder.indexOf(periodic.color_sensed.charAt(0)) == -1) {
            periodic.distance = 0;
        } else if (periodic.color_direction_calc == -3) {
            periodic.distance = -12.5;
        } else if (periodic.color_direction_calc == 3) {
            periodic.distance = 12.5;
        } else {
            periodic.distance = periodic.color_direction_calc * 12.5;
        }
    }
    public void setEnabled(boolean enabled)
    {
        periodic.isEnabled = enabled;
    }

    public void setCCW(boolean CCW)
    {
        periodic.CCW = CCW;
    }

    private void checkIfDone() {
        if (!(periodic.color_sensed.charAt(0) == 'U')) {
            if (!(periodic.color_sensed.charAt(0) == periodic.fms_color)) {
                periodic.demand = 12.0;
            } else {
                periodic.demand = 0;
            }
        } else {
            periodic.demand = .75;
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
        return periodic.fms_color == periodic.color_sensed.charAt(0);
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

        if (match.confidence <= .95)
            return "Unknown";

        if (match.color == Constants.kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == Constants.kRedTarget) {
            colorString = "Red";
        } else if (match.color == Constants.kGreenTarget) {
            colorString = "Green";
        } else if (match.color == Constants.kYellowTarget) {
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

    public char cDetected() {
        return periodic.color_sensed.charAt(0);
    }

    public LogData getLogger() {
        return periodic;

    }

    public class ColorWheelIO extends Subsystem.PeriodicIO {
        public boolean CCW = false;
        public boolean isEnabled = false;
        public double close_loop_error = 0;
        public char fms_color = 'U';
        public double distance = 0;
        public double[] RGB = new double[] { 0, 0, 0 };
        public int color_direction_calc;
        public double demand = 0.0;
        public Color detected_color = Color.kBlack;
        public String color_sensed = "Unknown";
        public boolean color_motor_pid_on = false;
        public boolean color_wheel_reading = false;
    }
}
