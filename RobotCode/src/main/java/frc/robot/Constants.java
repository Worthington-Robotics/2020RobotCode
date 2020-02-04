package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    /**
     * device ID declarations ---------------------------------
     */
    
    //Pigion ID
    public static final int PIGION_ID = 0;
    //Talon IDs
    public static final int SHOOTER_FLYWHEEL_RIGHT = 1;
    public static final int SHOOTER_FLYWHEEL_LEFT = 2;
    public static final int COLOR_WHEEL = 3;
    public static final int TURRET_CONTROL = 13;
    public static final int DRIVE_FRONT_LEFT_ID = 1;
    public static final int DRIVE_MIDDLE_LEFT_ID = 2;
    public static final int DRIVE_BACK_LEFT_ID = 3;
    public static final int DRIVE_FRONT_RIGHT_ID = 4;
    public static final int DRIVE_MIDDLE_RIGHT_ID = 5;
    public static final int DRIVE_BACK_RIGHT_ID = 6;

    //Spark Ports

    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 1;

    //Color Sensor Port
    public static final int COLOR_SENSOR_PORT = 0;
    
    //LED Data/Ports
    public static final int LED_PORT = 1;
    public static final int LED_LENGTH = 60;
    /*
        Flywheel Tuned Values
    */
    public static final double IDLE_RPM = 4000;
    public static final double RPM_ACCEPTIBLE_ERROR = 300; // Ticks per 100 ms

    /* 
        Turret tuned values ---------------
    */
    public static final double TURRET_MAX_SPEED = .75;
    public static final double TURRET_CONTROL_PID_P = 0;
    public static final double TURRET_CONTROL_PID_D = 0;

    //Joystick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick LAUNCH_PAD = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0.2, 0.99, 0.99, 0.6, 2);
    public static final HIDHelper.HIDConstants LAUNCHPAD_STICK = new HIDHelper.HIDConstants(LAUNCH_PAD, 0.2, 0.99, 0.99, 0.8, 2);

    /**
     * Drivetrain tuned values --------------------------------
     */

    //Encoder Constants
    public static final double ENCODER_5046_CPR = 1024;

    //Path Following Constants
    public static final double PATH_FOLLOWING_LOOKAHEAD = 0;
    public static final double DRIVETRAIN_UPDATE_RATE = 0;
    public static final double PATH_FOLLOWING_MAX_ACCELERATION = 0;
    public static final double DRIVE_MAX_VEL = 0; //in/s
    //Test Flags 
    public static final boolean RAMPUP = false;
    public static final double MP_TEST_SPEED = 72; //in/s
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

	public static double LEFTFLYWHEELFALCON_KD = 0.0;
    public static double LEFTFLYWHEELFALCON_KP = 0.0;
    public static double RIGHTFLYWHEELFALCON_KD = 0.0;
    public static double RIGHTFLYWHEELFALCON_KP = 0.0;

	//Color Wheel Constants
    public static final int COLOR_WHEEL_RED_HUE1 = 0;
    public static final int COLOR_WHEEL_RED_HUE2 = 360;
    public static final int COLOR_WHEEL_YELLOW_HUE = 60;
    public static final int COLOR_WHEEL_GREEN_HUE = 120;
    public static final int COLOR_WHEEL_BLUE_HUE = 180;
    public static final int COLOR_WHEEL_HUE_ERROR = 10;

    public static final int COLOR_WHEEL_SAT_LIMIT = 80;
    public static final int COLOR_WHEEL_VAL_LIMIT = 80;

    public static final double COLOR_WHEEL_ROTATION_DISTANCE = 3.2 * (16 * Math.PI);

    public static final double COLOR_WHEEL_SPINNER_DIA = 2.0;

    public static final double COLOR_WHEEL_KF = 0.0; //TODO TUNE PID
    public static final double COLOR_WHEEL_KP = 0.0;
    public static final double COLOR_WHEEL_KI = 0.0;
    public static final double COLOR_WHEEL_KD = 0.0;
    public static final double COLOR_WHEEL_VCOMP = 0.0;

    

    //Limelight Constants
    public static final double LIMELIGHT_DEG_FOV = 0.0; //TODO CALCULATE FOV
    
    public static final double CLIMBER_EPSILON_CONST = 10;


    public static final int redH1 = 0;
    public static final int redH2 = 360;
    public static final int yellowH = 60;
    public static final int greenH = 120;
    public static final int blueH = 180;
    public static final int error = 10;

    public static final int satLimit = 80;
    public static final int valLimit = 80;
}