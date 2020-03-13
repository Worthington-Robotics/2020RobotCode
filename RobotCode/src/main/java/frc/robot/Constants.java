package frc.robot;

import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.Joystick;
import frc.lib.util.HIDHelper;

public class Constants {
    public static final boolean DEBUG = true;

    /**
     * device ID declarations ---------------------------------
     */
    
    //Pigion ID
    public static final int PIGION_ID = 0;

    //Talon SRX & FX IDs
    public static final int DRIVE_FRONT_RIGHT_ID = 1;
    public static final int DRIVE_BACK_RIGHT_ID = 2;
    public static final int SHOOTER_FLYWHEEL_LEFT = 5;
    public static final int SHOOTER_FLYWHEEL_RIGHT = 6;

    public static final int ID_SUPER_DELIVERY_WHEEL = 7;
    public static final int ID_SUPER_INDEX1 = 8;
    public static final int ID_SUPER_INDEX2 = 9;
    public static final int ID_SUPER_INDEX3 = 10;
    public static final int ID_SUPER_INTAKE = 11;

    public static final int TURRET_CONTROL = 12;
    public static final int DRIVE_BACK_LEFT_ID = 14;
    public static final int DRIVE_FRONT_LEFT_ID = 15;

    // TOF IDs
    public static final int ID_SUPER_TOF1 = 1;
    public static final int ID_SUPER_TOF2 = 2;
    public static final int ID_SUPER_TOF3 = 3;
    public static final int ID_SUPER_TOF4 = 4;
    public static final int ID_SUPER_TOF5 = 5;

    //Solenoid Ports
    public static final int TRANS_LOW_ID = 0;
    public static final int TRANS_HIGH_ID = 7;
    
    public static final int UNFOLD_LOW_ID = 4;
    public static final int UNFOLD_HIGH_ID = 3;

    public static final int CLIMB_LOW_ID = 2; //Do Not Change
    public static final int CLIMB_HIGH_ID = 5; //Do Not ChangeS
    
    public static final int INTAKE_LOW_ID = 6;
    public static final int INTAKE_HIGH_ID = 1;
    //Color Sensor Port
    public static final int COLOR_SENSOR_PORT = 1;
    
    //LED Data/Ports
    public static final int LED_PORT = 0;
    public static final int LED_LENGTH = 36;
    //Joystick Constants
    public static final Joystick MASTER = new Joystick(0);
    public static final Joystick SECOND = new Joystick(1);
    public static final HIDHelper.HIDConstants MASTER_STICK = new HIDHelper.HIDConstants(MASTER, 0, 0.65, 0.65, 0.4, 2);
    public static final HIDHelper.HIDConstants MASTER_STICK_SHIFTED = new HIDHelper.HIDConstants(MASTER, 0, 0.45, 0.65, 0.4, 2);
    public static final HIDHelper.HIDConstants SECOND_STICK = new HIDHelper.HIDConstants(SECOND, 0.01, -0.33, 0.99, 0.8, 2);    

    /* 
        Turret & flywheel tuned values ------------------------
    */
    public static final double LIMELIGHT_HIGHT = 23;
    public static final double LIMELIGHT_PITCH = 31;
    public static final double TURRET_MAX_SPEED = .145;
    public static final double TURRET_ANGLE_KP = .8; // SAFE .3
    public static final double TURRET_ANGLE_KI = 0.003; // SAFE .001
    public static final double TURRET_ANGLE_KD = 50; // SAFE 12
    public static final double TURRET_OFFSET = 1.25; //1.25 VISION FINE TUNING 25ft
    public static final double TURRET_DEGREES_TO_TICKS = 85.26;
    public static final double TURRET_LOCKON_DELTA = 1.5;

    public static double TURRET_LEFT_FLY_KP = 0.08; //0.08
    public static double TURRET_LEFT_FLY_KD = 0.0; //0.00
    public static double TURRET_LEFT_FLY_KF = 0.048; //0.048

    public static double VOLTAGE_COMP_FLYWHEEL = 10;
    public static final double FLYWHEEL_DELTA_AMPS = 2.5;

    public static final double FLYWHEEL_RPM_PER_IN = 5; //4.4
    public static final double FLYWHEEL_BASE_RPM = 4350; //4000
    public static final double FLYWHEEL_SPINUP_TIME = 100; //10 ms 
    public static final double FLYWHEEL_IDLE_RPM = 5500; //RPM
    public static final double FLYWHEEL_MAX_RPM = 5900; //RPM
    public static final int FLYWHEEL_MIN_RPM_OFFSET = -200;
    public static final int FLYWHEEL_MAX_RPM_OFFSET = 600;
    public static final int FLYWHEEL_OFFSET_RPM_INCREMENT = 100;
    public static final double FLYWHEEL_TP100MS = 3.413;

    //Turret Encoder Limits
    public static final int leftTurretLimit = -7400;
    public static final int rightTurretLimit = 7400;

    /**
     *   Drivetrain tuned values ------------------------------
     */

    //DEBUG AND TESTING flags
    public static final boolean RAMPUP = false;
    public static final boolean ENABLE_MP_TEST_MODE = false;
    public static final double MP_TEST_SPEED = 72; //in/s

    public static double LOOPER_DT = 0.01;    
    
    //Physical Constants
    public static final double DRIVE_WHEEL_TRACK_WIDTH_INCHES = 21.75;
    public static final double DRIVE_WHEEL_DIAMETER_INCHES = 6.225; // 6
    public static final double DRIVE_WHEEL_RADIUS_INCHES = DRIVE_WHEEL_DIAMETER_INCHES / 2.0;
    public static final double TRACK_SCRUB_FACTOR = 1.0;  // TODO tune

    //Path following Constants
    public static final double ROBOT_MAX_VELOCITY = 120.0; // in/s
    public static final double ROBOT_MAX_ACCEL = 60.0; // in/s^2
    public static final double PATH_FOLLOWING_LOOKAHEAD = 24;
    public static final double DRIVETRAIN_UPDATE_RATE = LOOPER_DT;
    public static final double PATH_FOLLOWING_MAX_ACCELERATION = 60;

    //Electrical Constants
    public static final double DRIVE_VCOMP = 10.0; //V
    public static final double DRIVE_ENCODER_PPR = 33261.61; //Empir

    //PID Constants
    public static final double ANGLE_KP = -0.024; // 0.065;
    public static final double ANGLE_KI = 0; // 0.00125;
    public static final double ANGLE_KD = 0; // 0.1
    public static final double ANGLE_PID_EPISLON = 1;

    public static final double DRIVE_RIGHT_KP = .25;
    public static final double DRIVE_RIGHT_KI = 0.0;
    public static final double DRIVE_RIGHT_KD = 5; // 20 for practice bot
    public static final double DRIVE_RIGHT_KF = 0.065; //.485

    public static final double DRIVE_LEFT_KP = .25; // .0885
    public static final double DRIVE_LEFT_KI = 0.0; //NO INTEGRAL it masks deeper problems
    public static final double DRIVE_LEFT_KD = 5; //20 for practice
    public static final double DRIVE_LEFT_KF = 0.065;

    /**
     * Color Wheel Constants ---------------------------
     */
    //encoder constants
    public static final double COLOR_ENCODER_CPR = 1024;

    public static final double COLOR_WHEEL_ROTATION_DISTANCE = 3.2 * (16 * Math.PI);

    public static final double COLOR_WHEEL_SPINNER_DIA = 2.0;

    public static final double COLOR_WHEEL_KF = 0.0; //TODO TUNE PID
    public static final double COLOR_WHEEL_KP = 0.0;
    public static final double COLOR_WHEEL_KI = 0.0;
    public static final double COLOR_WHEEL_KD = 0.0;
    public static final double COLOR_WHEEL_VCOMP = 0.0;

    public static final Color kBlueTarget = ColorMatch.makeColor(0.128, 0.413, 0.459);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.172, 0.564, 0.264);
    public static final Color kRedTarget = ColorMatch.makeColor(0.498, 0.352, 0.150);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.315, 0.553, 0.132);    

    //Limelight Constants
    public static final double LIMELIGHT_DEG_FOV = 0.0; //TODO CALCULATE FOV
    public static final double fov = 0;
    public static final int redH1 = 0;
    public static final int redH2 = 360;
    public static final int yellowH = 60;
    public static final int greenH = 120;
    public static final int blueH = 180;
    public static final int error = 29;
    public static final int satLimit = 80;
    public static final int valLimit = 80;

    /**
     * Superstructure constants
     */
    public static final double DEMAND_STOP = 0;

    public static final double SUPER_DEMAND_INTAKE_MANUAL = 1; // FIXME WARNING: MUST BE DIFFERENT FROM THE DEFAULT INTAKE DEMAND (unless you want me to implement a boolean for manual control)
    public static final double SUPER_DEMAND_SHOOT = 1;
    
    /**
     * Climber constants
     */
    public static final double CLIMBER_SHOOTER_REQMT = 90;
    public static final double CLIMBER_EPSILON_CONST = 10;
}