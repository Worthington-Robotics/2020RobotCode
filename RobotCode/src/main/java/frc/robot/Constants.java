package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    /**
     * device ID declarations ---------------------------------
     */

    //Talon IDs
    public static final int SUPERSTRUCTURE_ABOVE_BELT = 7;
    public static final int SUPERSTRUCTURE_BELOW_BELT = 8;
    public static final int SUPERSTRUCTURE_DELIVERY_BELT = 9;
    public static final int SUPERSTRUCTURE_INTAKE = 10;
    //Spark Ports

    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;
    public static final int CLIMB_FRONT_LOW_ID = 2;
    public static final int CLIMB_FRONT_HIGH_ID = 3;
    public static final int LOCK_LOW_ID = 4;
    public static final int LOCK_HIGH_ID = 5;
    //Color Sensor Port
    public static final int COLOR_SENSOR_PORT = 0;
    
    //LED Data/Ports
    public static final int LED_PORT = 1;
    public static final int LED_LENGTH = 60;

    /**
     * Drivetrain tuned values --------------------------------
     */

    //Physical Constants
    public static final double DRIVE_WHEEL_TRACK_WIDTH_INCHES = 21.75;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.225; // 6
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.0;  // TODO tune
    public static final double ROBOT_LINEAR_INERTIA = 75;  // kg TODO tune
    public static final double ROBOT_ANGULAR_INERTIA = 10.0;  // kg m^2 TODO tune
    public static final double ROBOT_ANGULAR_DRAG = 12.0;  // N*m / (rad/sec) TODO tune

    //Path following Constants
    public static final double ROBOT_MAX_VELOCITY = 120.0; // in/s
    public static final double ROBOT_MAX_ACCEL = 120.0; // in/s^2
    public static final double ROBOT_MAX_VOLTAGE = 10.0; // V
    public static final double Path_Kx = 4.0;  //
    public static final double PATH_LOOK_AHEAD_TIME = 0.4;  // seconds to look ahead along the path for steering
    public static final double PATH_MIN_LOOK_AHEAD_DISTANCE = 24.0;  // inches

    //Electrical Constants
    public static final double DRIVE_V_INTERCEPT = 1.2;  // V //1.6 for practice......................
    public static final double DRIVE_Kv = 0.316426;  // V per rad/s -.335
    public static final double DRIVE_Ka = 0.0801;  // V per rad/s^2    0.0801
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double DRIVE_ENCODER_PPR = 4096.0; //encoder counts per revolution

    //PID Constants
    public static final double ANGLE_KP = 0.04; // 0.065;
    public static final double ANGLE_KI = 0; // 0.00125;
    public static final double ANGLE_KD = 0; // 0.1
    public static final double ANGLE_PID_EPISLON = 1;

    public static final double DRIVE_RIGHT_KP = 1.2;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 25; // 20 for practice bot
    public static final double DRIVE_RIGHT_KF = 0.53; //.485

    public static final double DRIVE_LEFT_KP = 1.1; // .0885
    public static final double DRIVE_LEFT_KI = 0.0; //NO INTEGRAL it masks deeper problems
    public static final double DRIVE_LEFT_KD = 25; //20 for practice
    public static final double DRIVE_LEFT_KF = 0.53;

	public static final double kPathFollowingMaxAccel = 0;

	public static final boolean ENABLE_MP_TEST_MODE = false;

	public static double LOOPER_DT = 0.01;

	//Color Wheel Constants
    public static final int redH1 = 0;
    public static final int redH2 = 360;
    public static final int yellowH = 60;
    public static final int greenH = 120;
    public static final int blueH = 180;
    public static final int error = 10;

    public static final int satLimit = 80;
    public static final int valLimit = 80;

    /**
     * Superstructure belt constants
     */
    // IDs
    public static final int FLIGHT_SENSOR_DELIVERY = 0;
    public static final int FLIGHT_SENSOR_INDEX = 1;
    public static final int FLIGHT_SENSOR_INTAKE = 2;
    // Demands
    public static final double FULL_BELT_DEMAND = 1;
    public static final double HIGH_BELT_DEMAND = 0.75;
    public static final double STOP_BELT_DEMAND = 0;
    // Sensor distance before stop (in mm)
    public static final double DISTANCE_STOP_MM = 25.4;
    public static final double DISTANCE_EMPTY_MM = 1219.2;

    /**
     * Joystick
     */
    //Stick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick LAUNCH_PAD = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.2, 0.99, 0.99, 0.6, 2);
    public static final HIDHelper.HIDConstants LAUNCHPAD_STICK = new HIDHelper.HIDConstants(LAUNCH_PAD, 0.2, 0.99, 0.99, 0.8, 2);
}