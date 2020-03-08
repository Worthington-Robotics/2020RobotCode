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

    private boolean[] manualControl = new boolean[5];
    private TalonSRX[] motors = new TalonSRX[5];
    private SimTimeOfFlight[] sensors = new SimTimeOfFlight[5];
    private double[] defaultMotorDemands = new double[] {
            // TODO Move to constants once done debugging demands
            .88, // BLACK_WHEEL - needs to go slow or will shoot...
            .5, // INDEXER_ONE
            .5, // INDEXER_TWO
            .5, // INDEXER_THREE
            .5 // INTAKE
    };
    private double[] purgeDemands = new double[] {
            -1, // BLACK_WHEEL
            -1, // INDEXER_ONE
            -1, // INDEXER_TWO
            -1, // INDEXER_THREE
            -1 // INTAKE
    };

    // Constants
    public static final short BLACK_WHEEL = 0;
    public static final short INDEXER_ONE = 1;
    public static final short INDEXER_TWO = 2;
    public static final short INDEXER_THREE = 3;
    public static final short INTAKE = 4;
    // TODO Move to constants once done debugging thresholds
    // (millimeters)
    public static double[] threshold = {
            45, // BLACK_WHEEL
            100, // INDEXER_ONE
            75, // INDEXER_TWO
            75, // INDEXER_THREE
            75 // INTAKE
    };

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
        // Set motor demands
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
                // Ignore motor setting if dumping
                if (periodic.state == SuperState.DEFAULT) {
                    // If in shoot mode, shoot
                    if (manualControl[BLACK_WHEEL]) {
                        if (periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.motorDemands[BLACK_WHEEL] = Constants.SUPER_DEMAND_SHOOT;
                        }
                    }
                    // // Not manual control
                    // } else if (periodic.sensorsDetected[BLACK_WHEEL]) {
                    //     periodic.motorDemands[BLACK_WHEEL] = Constants.DEMAND_STOP;
                    // } else if (periodic.sensorsDetected[INDEXER_ONE]) {
                    //     periodic.motorDemands[BLACK_WHEEL] = defaultMotorDemands[BLACK_WHEEL];
                    // }

                    // Iterate over all EXCEPT Intake
                    for (int n = INDEXER_ONE; n <= INTAKE; n++) {
                        if (!manualControl[n]) {
                            // If Ball n detected and Ball n-1 not detected
                            // periodic.motorDemands[n] = periodic.sensorsDetected[n] && !periodic.sensorsDetected[n - 1]
                            //         ? defaultMotorDemands[n] : Constants.DEMAND_STOP;

                            // FIXME New logic to deal with dead sensor zones
                            // if (periodic.sensorsDetected[n] && !periodic.sensorsDetected[n - 1]) {
                            //     periodic.motorDemands[n] = defaultMotorDemands[n];
                            //     periodic.motorDemands[n - 1] = defaultMotorDemands[n - 1];
                            // } else {
                            //     if (!periodic.sensorsDetected[n]) {
                            //         periodic.motorDemands[n] = Constants.DEMAND_STOP;
                            //     }
                            //     if (periodic.sensorsDetected[n - 1]) {
                            //         periodic.motorDemands[n - 1] = Constants.DEMAND_STOP;
                            //     }
                            // }
                        }
                    }

                    if (!manualControl[BLACK_WHEEL]) {
                        if (periodic.sensorsDetected[INDEXER_ONE] && !periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.motorDemands[BLACK_WHEEL] = defaultMotorDemands[BLACK_WHEEL];
                        } else if (periodic.sensorsDetected[BLACK_WHEEL]) {
                            periodic.motorDemands[BLACK_WHEEL] = Constants.DEMAND_STOP;
                        }
                    }
                    
                    if (periodic.sensorsDetected[INDEXER_ONE] && !periodic.sensorsDetected[BLACK_WHEEL] ||
                        periodic.sensorsDetected[INDEXER_TWO] && !periodic.sensorsDetected[INDEXER_ONE]) {
                        periodic.motorDemands[INDEXER_ONE] = defaultMotorDemands[INDEXER_ONE];
                    } else if (!periodic.sensorsDetected[INDEXER_ONE]) {
                        periodic.motorDemands[INDEXER_ONE] = Constants.DEMAND_STOP;
                    }

                    if (periodic.sensorsDetected[INDEXER_TWO] && !periodic.sensorsDetected[INDEXER_ONE] ||
                        periodic.sensorsDetected[INDEXER_THREE] && !periodic.sensorsDetected[INDEXER_TWO]) {
                        periodic.motorDemands[INDEXER_TWO] = defaultMotorDemands[INDEXER_TWO];
                    } else if (!periodic.sensorsDetected[INDEXER_TWO]) {
                        periodic.motorDemands[INDEXER_TWO] = Constants.DEMAND_STOP;
                    }

                    if (periodic.sensorsDetected[INDEXER_THREE] && !periodic.sensorsDetected[INDEXER_TWO] ||
                        periodic.sensorsDetected[INTAKE] && !periodic.sensorsDetected[INDEXER_THREE]) {
                        periodic.motorDemands[INDEXER_THREE] = defaultMotorDemands[INDEXER_THREE];
                    } else if (!periodic.sensorsDetected[INDEXER_THREE]) {
                        periodic.motorDemands[INDEXER_THREE] = Constants.DEMAND_STOP;
                    }

                    if (!manualControl[INTAKE]) {
                        if (periodic.sensorsDetected[INTAKE] && !periodic.sensorsDetected[INDEXER_THREE]) {
                        periodic.motorDemands[INTAKE] = defaultMotorDemands[INTAKE];
                    } else if (!periodic.sensorsDetected[INTAKE]) {
                        periodic.motorDemands[INTAKE] = Constants.DEMAND_STOP;
                    }
                    }
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

    /**
     * Sets the arm extension of the intake.
     * @param armExtension if the intake arm is extended
     */
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    }

    /**
     * Sets the new state and performs a task based on the new state.
     * @param newState the new state of the subsystem
     */
    public void setState(SuperState newState) {
        periodic.state = newState;

        switch (periodic.state) {
            case DUMP: {
                for (int n = BLACK_WHEEL; n <= INTAKE; n++) {
                    if (!manualControl[n]) {
                        // Opposite and flip the default motor demands so the back is faster
                        periodic.motorDemands[n] = purgeDemands[n];
                    }
                }
                break;
            }
            default: {
                if (!manualControl[BLACK_WHEEL]) {
                    // Stop wheel manually because the non-dumping mode does not override it automatically
                    periodic.motorDemands[BLACK_WHEEL] = Constants.DEMAND_STOP;
                }
            }
        }
    }

    public void setShooting(boolean shooting) {
        manualControl[BLACK_WHEEL] = shooting;
    }

    public void setIntaking(boolean intaking) {
        manualControl[INTAKE] = intaking;
        periodic.motorDemands[INTAKE] = intaking ? Constants.SUPER_DEMAND_INTAKE_MANUAL : Constants.DEMAND_STOP;
    }

    /**
     * Stores data for the ReflectingLogger to grab and dump.
     */
    public class SuperIO extends Subsystem.PeriodicIO {
        public boolean[] sensorsDetected = new boolean[5];
        public double[] motorDemands = new double[5];

        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;

        public SuperState state = SuperState.DEFAULT;
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

        for (int n = INDEXER_ONE; n < INTAKE; n++) {
            motors[n].configSupplyCurrentLimit(
                    new SupplyCurrentLimitConfiguration(true, 5, 5, .2));
        }

        for (TalonSRX motor : motors) {
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    public enum SuperState {
        DEFAULT, DUMP
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
