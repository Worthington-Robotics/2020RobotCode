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
public class SuperstructureNew extends Subsystem {
    private SuperIO periodic;
    private DoubleSolenoid extensionArm;

    private TalonSRX[] motors = new TalonSRX[5];
    private SimTimeOfFlight[] sensors = new SimTimeOfFlight[5];

    // Constants
    private static final short BLACK_WHEEL = 0;
    private static final short INDEXER_ONE = 1;
    private static final short INDEXER_TWO = 2;
    private static final short INDEXER_THREE = 3;
    private static final short INTAKE = 4;
    // (in mm)
    public static double SUPER_TOF1_THRESHOLD = 75;
    public static double SUPER_TOF2_THRESHOLD = 75;
    public static double SUPER_TOF3_THRESHOLD = 75;
    public static double SUPER_TOF4_THRESHOLD = 75;
    public static double SUPER_TOF5_THRESHOLD = 75;

    private static SuperstructureNew instance = new SuperstructureNew();
    public static SuperstructureNew getInstance() {
        return instance;
    }

    /**
     * Initializes motors, sensors, and the intake's DoubleSolenoid arm. Also
     * resets IO, motors, and sensors to default functioning settings.
     */
    public SuperstructureNew() {
        motors[0] = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        motors[1] = new TalonSRX(Constants.ID_SUPER_INDEX1);
        motors[2] = new TalonSRX(Constants.ID_SUPER_INDEX2);
        motors[3] = new TalonSRX(Constants.ID_SUPER_INDEX3);
        motors[4] = new TalonSRX(Constants.ID_SUPER_INTAKE);

        sensors[0] = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        sensors[1] = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        sensors[2] = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        sensors[3] = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        sensors[4] = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);

        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        reset();
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
                for (int n = BLACK_WHEEL; n < INTAKE; n++) {
                    // Ensure that the Intake is not in manual control before auto-moving
                    if (n != INTAKE || periodic.motorDemands[INTAKE] != Constants.SUPER_DEMAND_INTAKE_MANUAL) {
                        // If Ball n detected and Ball n-1 not detected
                        periodic.motorDemands[n] = periodic.sensorsDetected[n] && !periodic.sensorsDetected[n - 1] ?
                                Constants.SUPER_DEMAND_DEFAULT : Constants.SUPER_DEMAND_STOP;
                    }
                }
            }

            @Override public void onStart(double timestamp) {}
            @Override public void onStop(double timestamp) {}
        });
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
     * Sets the arm extension of the intake.
     * @param armExtension intake arm is extended
     */
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    }

    /**
     * Stores data for the ReflectingLogger to grab and dump.
     */
    public class SuperIO extends Subsystem.PeriodicIO {
        public boolean[] sensorsDetected = new boolean[5];
        public double[] motorDemands = new double[5];

        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;
    }

    /**
     * Outputs and reads all logging information to/from the SmartDashboard.
     */
    @Override public void outputTelemetry() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/TOF1_DISTANCE", sensors[0].getRange());
            SmartDashboard.putNumber("Superstructure/TOF2_DISTANCE", sensors[1].getRange());
            SmartDashboard.putNumber("Superstructure/TOF3_DISTANCE", sensors[2].getRange());
            SmartDashboard.putNumber("Superstructure/TOF4_DISTANCE", sensors[3].getRange());
            SmartDashboard.putNumber("Superstructure/TOF5_DISTANCE", sensors[4].getRange());

            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF1", SUPER_TOF1_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF2", SUPER_TOF2_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF3", SUPER_TOF3_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF4", SUPER_TOF4_THRESHOLD);
            SUPER_TOF2_THRESHOLD = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF5", SUPER_TOF5_THRESHOLD);

            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF1", SUPER_TOF1_THRESHOLD);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF2", SUPER_TOF2_THRESHOLD);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF3", SUPER_TOF3_THRESHOLD);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF4", SUPER_TOF4_THRESHOLD);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF5", SUPER_TOF5_THRESHOLD);

            SmartDashboard.putNumber("Superstructure/BLACK_WHEEL_DEMAND", periodic.motorDemands[BLACK_WHEEL]);
            SmartDashboard.putNumber("Superstructure/INDEXER1_DEMAND", periodic.motorDemands[INDEXER_ONE]);
            SmartDashboard.putNumber("Superstructure/INDEXER2_DEMAND", periodic.motorDemands[INDEXER_TWO]);
            SmartDashboard.putNumber("Superstructure/INDEXER3_DEMAND", periodic.motorDemands[INDEXER_THREE]);
            SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.motorDemands[INTAKE]);
        }
    }
}
