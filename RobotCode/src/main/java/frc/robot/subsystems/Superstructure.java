package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
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
    // TODO Move to constants once done debugging demands
    private double[] defaultMotorDemands = new double[] {
            .4, // BLACK_WHEEL - needs to go slow or will shoot...
            .5, // INDEXER_ONE
            .5, // INDEXER_TWO
            .5, // INDEXER_THREE
            .5 // INTAKE
    };
    // TODO Move to constants once done debugging thresholds
    private double[] purgeDemands = new double[] {
            -1, // BLACK_WHEEL
            -1, // INDEXER_ONE
            -1, // INDEXER_TWO
            -1, // INDEXER_THREE
            -1 // INTAKE
    };
    // TODO Move to constants once done debugging thresholds
    // (millimeters)
    public static double[] threshold = {
            75, // BLACK_WHEEL
            75, // INDEXER_ONE
            75, // INDEXER_TWO
            75, // INDEXER_THREE
            75 // INTAKE
    };

    // Constants
    public static final short BLACK_WHEEL = 0;
    public static final short INDEXER_ONE = 1;
    public static final short INDEXER_TWO = 2;
    public static final short INDEXER_THREE = 3;
    public static final short INTAKE = 4;

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
        configTalons();

        // Put value on dashboard so it appears and can be modified
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF1", threshold[BLACK_WHEEL]);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF2", threshold[INDEXER_ONE]);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF3", threshold[INDEXER_TWO]);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF4", threshold[INDEXER_THREE]);
        SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF5", threshold[INTAKE]);

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
        for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
            periodic.sensorsDetected[n] = sensorDetected(n);
        }
    }

    /**
     * Writes the periodic outputs to actuators (motors, etc.).
     */
    @Override public void writePeriodicOutputs() {
        for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
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
                switch (periodic.state) {
                    // TODO Simplify
                    case DISABLED:
                    case INIT:
                        if (periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.ONE_BALL;
                        }
                        break;

                    case ONE_BALL:
                        if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.INIT;
                        }
                        if (periodic.sensorsDetected[INDEXER_ONE]) {
                            periodic.state = SuperState.TWO_BALLS;
                        }
                        break;

                    case TWO_BALLS:
                        if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.INIT;
                        }
                        if (periodic.sensorsDetected[INDEXER_TWO]) {
                            periodic.state = SuperState.THREE_BALLS;
                        }
                        break;

                    case THREE_BALLS:
                        if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.INIT;
                        }
                        if (periodic.sensorsDetected[INDEXER_THREE]) {
                            periodic.state = SuperState.FOUR_BALLS;
                        }
                        break;

                    case FOUR_BALLS:
                        if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.INIT;
                        }
                        if (periodic.sensorsDetected[INTAKE]) {
                            periodic.state = SuperState.FULL_SYSTEM;
                        }
                        break;

                    case FULL_SYSTEM:
                        if (!periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.state = SuperState.INIT;
                        }
                        if (!periodic.sensorsDetected[INTAKE]) {
                            periodic.state = SuperState.FOUR_BALLS;
                        }
                        break;

                    case SHOOT:
                        break;
                    default:
                }

                switch (periodic.state) {
                    case INIT:
                        // Set all motors to their DEFAULT demands
                        System.arraycopy(defaultMotorDemands, BLACK_WHEEL, periodic.motorDemands, BLACK_WHEEL, INTAKE);
                        break;
                    case ONE_BALL: case TWO_BALLS: case THREE_BALLS: case FOUR_BALLS: case FULL_SYSTEM:
                        periodic.motorDemands[periodic.state.getID()] = Constants.DEMAND_STOP;
                        break;
                    case SHOOT:
                        periodic.motorDemands[BLACK_WHEEL] = Constants.SUPER_DEMAND_SHOOT;
                        break;
                    case DUMP_SYSTEM:
                        // Set all motors to their PURGE demands
                        System.arraycopy(purgeDemands, BLACK_WHEEL, periodic.motorDemands, BLACK_WHEEL, INTAKE + 1);
                        break;
                    default:
                }
            }

            @Override public void onStart(double timestamp) {}
            @Override public void onStop(double timestamp) {}
        });
    }

    public boolean isSystemEmpty() {
        for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
            if (periodic.sensorsDetected[n]) {
                return false;
            }
        }
        return true;
    }

    public boolean sensorDetected(int id) {
        return sensors[id].getRange() != 0 && sensors[id].getRange() < threshold[id];
    }

    public void shootBall() {
        if (periodic.sensorsDetected[BLACK_WHEEL]) {
            periodic.state = SuperState.SHOOT;
        }
    }

    /**
     * Sets the arm extension of the intake.
     * @param armExtension if the intake arm is extended
     */
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    }

    public void setIntaking(boolean intaking) {
        // Enable the state system
        if (periodic.state == SuperState.DISABLED) {
            periodic.state = SuperState.INIT;
        }

        if (periodic.state != SuperState.FULL_SYSTEM && intaking) {
            periodic.motorDemands[INTAKE] = Constants.SUPER_DEMAND_INTAKE_MANUAL;
        } else {
            periodic.motorDemands[INTAKE] = Constants.DEMAND_STOP;
        }

    }

    public void setDumping(boolean dumping) {
        periodic.state = dumping ? SuperState.DUMP_SYSTEM : SuperState.INIT;
    }

    /**
     * Stores data for the ReflectingLogger to grab and dump.
     */
    public class SuperIO extends Subsystem.PeriodicIO {
        public boolean[] sensorsDetected = new boolean[5];
        public double[] motorDemands = new double[5];

        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;

        public SuperState state = SuperState.DISABLED;
    }
    public LogData getLogger() {
        return periodic;
    }

    /**
     * Resets and configures the subsystem.
     */
    @Override public void reset() {
        periodic = new SuperIO();

        motors[BLACK_WHEEL].setInverted(true);

        for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
            sensors[n].setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        }
    }

    private void configTalons() {
        motors[BLACK_WHEEL].configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 15, 15, .2));
        motors[INDEXER_ONE].configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 5, 5, .2));
        motors[INDEXER_TWO].configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 10, 10, .2));
        motors[INDEXER_THREE].configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 5, 5, .2));
        motors[INTAKE].configSupplyCurrentLimit(
                new SupplyCurrentLimitConfiguration(true, 5, 5, .2));

        for (TalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    public enum SuperState {
        DISABLED(-2),
        INIT(-1),

        ONE_BALL(BLACK_WHEEL),
        TWO_BALLS(INDEXER_ONE),
        THREE_BALLS(INDEXER_TWO),
        FOUR_BALLS(INDEXER_THREE),
        FULL_SYSTEM(INTAKE),

        SHOOT(5),
        DUMP_SYSTEM(6);

        private int id;

        SuperState(int id) {
            this.id = id;
        }

        public int getID() {
            return id;
        }

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }

    /**
     * Outputs and reads all logging information to/from the SmartDashboard.
     */
    @Override public void outputTelemetry() {
        if (Constants.DEBUG) {
            threshold[BLACK_WHEEL] = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF1", threshold[BLACK_WHEEL]);
            threshold[INDEXER_ONE] = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF2", threshold[INDEXER_ONE]);
            threshold[INDEXER_TWO] = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF3", threshold[INDEXER_TWO]);
            threshold[INDEXER_THREE] = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF4", threshold[INDEXER_THREE]);
            threshold[INTAKE] = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF5", threshold[INTAKE]);

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

            SmartDashboard.putString("Superstructure/STATE", periodic.state.toString());
        }
        // FIXME Name differently if possible after changing driver station value
        SmartDashboard.putBoolean("Superstructure/BALL1", periodic.sensorsDetected[BLACK_WHEEL]);
        SmartDashboard.putBoolean("Superstructure/BALL2", periodic.sensorsDetected[INDEXER_ONE]);
        SmartDashboard.putBoolean("Superstructure/BALL3", periodic.sensorsDetected[INDEXER_TWO]);
        SmartDashboard.putBoolean("Superstructure/BALL4", periodic.sensorsDetected[INDEXER_THREE]);
        SmartDashboard.putBoolean("Superstructure/BALL5", periodic.sensorsDetected[INTAKE]);
    }
}
