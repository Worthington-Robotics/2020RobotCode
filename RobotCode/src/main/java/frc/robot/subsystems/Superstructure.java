package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.robot.Constants;

/**
 * The Superstructure subsystem combines both the
 * Indexer and Intake systems for performance economy.
 *
 * This version of the Superstructure is designed for the second
 * design iteration of the robot, specifically noting the new top
 * wheels used in the Indexer.
 *
 * @author Spazzinq (George Fang)
 */
public class Superstructure extends Subsystem {
    private SuperIO periodic;
    private DoubleSolenoid extensionArm;

    private TalonSRX[] motors = new TalonSRX[5];
    private SimTimeOfFlight[] sensors = new SimTimeOfFlight[5];
    private double[] defaultMotorDemands = new double[] {
            // TODO Move to constants once done debugging demands
            .2, // BLACK_WHEEL - needs to go slow or will shoot...
            .5, // INDEXER_ONE
            .45, // INDEXER_TWO
            .4, // INDEXER_THREE
            .35, // INTAKE
    };
    private boolean dumping;

    // Constants
    public static final short BLACK_WHEEL = 0;
    public static final short INDEXER_ONE = 1;
    public static final short INDEXER_TWO = 2;
    public static final short INDEXER_THREE = 3;
    public static final short INTAKE = 4;
    // TODO Move to constants once done debugging thresholds
    // (in mm)
    public static double SUPER_TOF1_THRESHOLD = 75;
    public static double SUPER_TOF2_THRESHOLD = 75;
    public static double SUPER_TOF3_THRESHOLD = 75;
    public static double SUPER_TOF4_THRESHOLD = 75;
    public static double SUPER_TOF5_THRESHOLD = 75;

    private static Superstructure instance = new Superstructure();
    public static Superstructure getInstance() {
        return instance;
    }

    /**
     * Initializes motors, sensors, and the intake's DoubleSolenoid arm. Also
     * resets IO, motors, and sensors to default functioning settings.
     */
    public Superstructure() {
        motors[BLACK_WHEEL] = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        motors[INDEXER_ONE] = new TalonSRX(Constants.ID_SUPER_INDEX1);
        motors[INDEXER_TWO] = new TalonSRX(Constants.ID_SUPER_INDEX2);
        motors[INDEXER_THREE] = new TalonSRX(Constants.ID_SUPER_INDEX3);
        motors[INTAKE] = new TalonSRX(Constants.ID_SUPER_INTAKE);

        sensors[BLACK_WHEEL] = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        sensors[INDEXER_ONE] = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        sensors[INDEXER_TWO] = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        sensors[INDEXER_THREE] = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        sensors[INTAKE] = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);

        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        reset();
        // Put value on dashboard so it appears and can be modified
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF1", SUPER_TOF1_THRESHOLD);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF2", SUPER_TOF2_THRESHOLD);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF3", SUPER_TOF3_THRESHOLD);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF4", SUPER_TOF4_THRESHOLD);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF5", SUPER_TOF5_THRESHOLD);

        SmartDashboard.putNumber("Superstructure/DEMAND_BLACK_WHEEL_DEFAULT", defaultMotorDemands[BLACK_WHEEL]);
        SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER1_DEFAULT", defaultMotorDemands[INDEXER_ONE]);
        SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER2_DEFAULT", defaultMotorDemands[INDEXER_TWO]);
        SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER3_DEFAULT", defaultMotorDemands[INDEXER_THREE]);
        SmartDashboard.putNumber("Superstructure/DEMAND_INTAKE_DEFAULT", defaultMotorDemands[INTAKE]);
    }

    /**
     * Updates all periodic variables and sensors.
     */
    @Override public void readPeriodicInputs() {
        periodic.sensorsDetected = new boolean[] {
                sensors[BLACK_WHEEL].getRange() != 0 && sensors[BLACK_WHEEL].getRange() < SUPER_TOF1_THRESHOLD,
                sensors[INDEXER_ONE].getRange() != 0 && sensors[INDEXER_ONE].getRange() < SUPER_TOF2_THRESHOLD,
                sensors[INDEXER_TWO].getRange() != 0 && sensors[INDEXER_TWO].getRange() < SUPER_TOF3_THRESHOLD,
                sensors[INDEXER_THREE].getRange() != 0 && sensors[INDEXER_THREE].getRange() < SUPER_TOF4_THRESHOLD,
                sensors[INTAKE].getRange() != 0 && sensors[INTAKE].getRange() < SUPER_TOF5_THRESHOLD,
        };
    }

    /**
     * Writes the periodic outputs to actuators (motors, etc.).
     */
    @Override public void writePeriodicOutputs() {
        // Set motor demands
        for (int n = BLACK_WHEEL; n < INTAKE; n++) {
            motors[n].set(ControlMode.PercentOutput, periodic.motorDemands[n]);
        }

        extensionArm.set(periodic.armExtension);
    }

    /**
     * Registers Loops to loop while the robot is active.
     * @param enabledLooper the subsystem's Looper
     */
    @Override public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override public void onLoop(double timestamp) {
                // When ball is no longer detected after shot
                if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                    periodic.motorDemands[BLACK_WHEEL] = Constants.SUPER_DEMAND_STOP;
                }

                // Ignore motor setting if dumping and automatically disable once empty
                if (dumping) {
                    boolean empty = true;

                    for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
                        if (periodic.sensorsDetected[n]) {
                            empty = false;
                            break;
                        }
                    }

                    if (empty) {
                        dumping = false;
                    }
                } else {
                    for (int n = INDEXER_ONE; n <= INTAKE; n++) {
                        // Ensure that the Intake is not in manual control before auto-moving
                        if (n != INTAKE || periodic.motorDemands[INTAKE] != Constants.SUPER_DEMAND_INTAKE_MANUAL) {
                            // If Ball n detected and Ball n-1 not detected
                            periodic.motorDemands[n] = periodic.sensorsDetected[n] && !periodic.sensorsDetected[n - 1] ?
                                    defaultMotorDemands[n] : Constants.SUPER_DEMAND_STOP;
                        }
                    }
                }
            }

            @Override public void onStart(double timestamp) {}
            @Override public void onStop(double timestamp) {}
        });
    }

    /**
     * Enables the black wheel to shoot a ball.
     */
    public void shootBall() {
        if (periodic.sensorsDetected[BLACK_WHEEL]) {
            periodic.motorDemands[BLACK_WHEEL] = Constants.SUPER_DEMAND_SHOOT;
        }
    }

    /**
     * Sets the arm extension of the intake.
     * @param armExtension if the intake arm is extended
     */
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    }

    /**
     * Sets a motor to a demand.
     * @param id the motor ID
     * @param demand the demand
     */
    public void setMotorDemand(int id, double demand) {
        periodic.motorDemands[id] = demand;
    }

    /**
     * If true, set all motor demands to negative in order to dump the balls.
     * @param dumping if system should dump balls
     */
    public void setDumping(boolean dumping) {
        this.dumping = dumping;

        if (dumping) {
            for (int n = INTAKE; n >= BLACK_WHEEL; n--) {
                if (n != INTAKE || periodic.motorDemands[INTAKE] != Constants.SUPER_DEMAND_INTAKE_MANUAL) {
                    // Opposite and flip the default motor demands so the back is faster
                    periodic.motorDemands[n] = -defaultMotorDemands[defaultMotorDemands.length - n];
                }
            }
        }
    }

    /**
     * Stores data for the ReflectingLogger to grab and dump.
     */
    public class SuperIO extends Subsystem.PeriodicIO {
        public boolean[] sensorsDetected = new boolean[5];
        public double[] motorDemands = new double[5];

        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;
    }
    public LogData getLogger() {
        return periodic;
    }

    /**
     * Called to reset and configure the subsystem.
     */
    @Override public void reset() {
        periodic = new SuperIO();

        motors[BLACK_WHEEL].setInverted(true);

        for (int n = BLACK_WHEEL; n < INTAKE; n++) {
            sensors[n].setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        }
    }

    /**
     * Outputs and reads all logging information to/from the SmartDashboard.
     */
    @Override public void outputTelemetry() {
        if (Constants.DEBUG) {
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF1", SUPER_TOF1_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF2", SUPER_TOF2_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF3", SUPER_TOF3_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF4", SUPER_TOF4_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF5", SUPER_TOF5_THRESHOLD);

            defaultMotorDemands[BLACK_WHEEL] = SmartDashboard.getNumber("Superstructure/DEMAND_BLACK_WHEEL_DEFAULT", defaultMotorDemands[BLACK_WHEEL]);
            defaultMotorDemands[INDEXER_ONE] = SmartDashboard.getNumber("Superstructure/DEMAND_INDEXER1_DEFAULT", defaultMotorDemands[INDEXER_ONE]);
            defaultMotorDemands[INDEXER_TWO] = SmartDashboard.getNumber("Superstructure/DEMAND_INDEXER2_DEFAULT", defaultMotorDemands[INDEXER_TWO]);
            defaultMotorDemands[INDEXER_THREE] = SmartDashboard.getNumber("Superstructure/DEMAND_INDEXER3_DEFAULT", defaultMotorDemands[INDEXER_THREE]);
            defaultMotorDemands[INTAKE] = SmartDashboard.getNumber("Superstructure/DEMAND_INTAKE_DEFAULT", defaultMotorDemands[INTAKE]);

            SmartDashboard.putNumber("Superstructure/DISTANCE_TOF1_RAW", sensors[BLACK_WHEEL].getRange());
            SmartDashboard.putNumber("Superstructure/DISTANCE_TOF2_RAW", sensors[INDEXER_ONE].getRange());
            SmartDashboard.putNumber("Superstructure/DISTANCE_TOF3_RAW", sensors[INDEXER_TWO].getRange());
            SmartDashboard.putNumber("Superstructure/DISTANCE_TOF4_RAW", sensors[INDEXER_THREE].getRange());
            SmartDashboard.putNumber("Superstructure/DISTANCE_TOF5_RAW", sensors[INTAKE].getRange());

            SmartDashboard.putNumber("Superstructure/DEMAND_BLACK_WHEEL_RAW", periodic.motorDemands[BLACK_WHEEL]);
            SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER1_RAW", periodic.motorDemands[INDEXER_ONE]);
            SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER2_RAW", periodic.motorDemands[INDEXER_TWO]);
            SmartDashboard.putNumber("Superstructure/DEMAND_INDEXER3_RAW", periodic.motorDemands[INDEXER_THREE]);
            SmartDashboard.putNumber("Superstructure/DEMAND_INTAKE_RAW", periodic.motorDemands[INTAKE]);
        }
        // FIXME Name differently if possible after changing driver station value
        SmartDashboard.putBoolean("Superstructure/BALL1", periodic.sensorsDetected[BLACK_WHEEL]);
        SmartDashboard.putBoolean("Superstructure/BALL2", periodic.sensorsDetected[INDEXER_ONE]);
        SmartDashboard.putBoolean("Superstructure/BALL3", periodic.sensorsDetected[INDEXER_TWO]);
        SmartDashboard.putBoolean("Superstructure/BALL4", periodic.sensorsDetected[INDEXER_THREE]);
        SmartDashboard.putBoolean("Superstructure/BALL5", periodic.sensorsDetected[INTAKE]);
    }
}
