package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.TimerBoolean;
import frc.robot.Constants;

/**
 * Combines both the indexer and intake functions for performance economy.
 */
public class Superstructure extends Subsystem {

    private static Superstructure instance = new Superstructure();

    public static Superstructure getInstance() {
        return instance;
    }

    // Global
    private SuperIO periodic;
    private TimerBoolean indexBoolean = new TimerBoolean(Constants.TIME_TILL_STATIONARY);

    // Indexer
    private TalonSRX deliveryWheel;
    private TalonSRX indexTopBelt;

    // Intake
    private TalonSRX intakeWheels;
    private DoubleSolenoid extensionArm;

    // Sensors
    private SimTimeOfFlight tof1;
    private SimTimeOfFlight tof2;
    private SimTimeOfFlight tof3;
    private SimTimeOfFlight tof4;
    private SimTimeOfFlight tof5;

    private double THRESHOLD_TOF1 = 50;
    private double THRESHOLD_TOF2 = 75;
    private double THRESHOLD_TOF3 = 75;
    private double THRESHOLD_TOF4 = 75;
    private double THRESHOLD_TOF5 = 75;

    private Superstructure() {
        deliveryWheel = new TalonSRX(Constants.ID_SUPER_DELIVERY_WHEEL);
        indexTopBelt = new TalonSRX(Constants.ID_SUPER_INDEX1);

        intakeWheels = new TalonSRX(Constants.ID_SUPER_INTAKE);
        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        tof1 = new SimTimeOfFlight(Constants.ID_SUPER_TOF1);
        tof2 = new SimTimeOfFlight(Constants.ID_SUPER_TOF2);
        tof3 = new SimTimeOfFlight(Constants.ID_SUPER_TOF3);
        tof4 = new SimTimeOfFlight(Constants.ID_SUPER_TOF4);
        tof5 = new SimTimeOfFlight(Constants.ID_SUPER_TOF5);

        reset();

        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/TOF_1_DISTANCE", tof1.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_2_DISTANCE", tof2.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_3_DISTANCE", tof3.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_4_DISTANCE", tof4.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_5_DISTANCE", tof5.getRange());
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF1", THRESHOLD_TOF1);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF2", THRESHOLD_TOF2);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF3", THRESHOLD_TOF3);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF4", THRESHOLD_TOF4);
            SmartDashboard.putNumber("Superstructure/THRESHOLD_TOF5", THRESHOLD_TOF5);
        }
    }

    /**
     * Read data from the sensors
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/TOF_1_DISTANCE", tof1.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_2_DISTANCE", tof2.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_3_DISTANCE", tof3.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_4_DISTANCE", tof4.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_5_DISTANCE", tof5.getRange());
            THRESHOLD_TOF1 = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF1", THRESHOLD_TOF1);
            THRESHOLD_TOF2 = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF2", THRESHOLD_TOF2);
            THRESHOLD_TOF3 = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF3", THRESHOLD_TOF3);
            THRESHOLD_TOF4 = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF4", THRESHOLD_TOF4);
            THRESHOLD_TOF5 = SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF5", THRESHOLD_TOF5);
        }

        /*
         * final double distanceDelivery = deliverySensor.getRange(); final double
         * distanceIndexer = indexSensor.getRange(); final double distanceIntake =
         * intakeSensor.getRange();
         */

        periodic.indexBeltAmps = indexTopBelt.getSupplyCurrent();
        /*
         * periodic.deliveryDetected = distanceDelivery != 0 && THRESHOLD_DELIVERY >=
         * distanceDelivery; periodic.indexDetected = distanceIndexer != 0 &&
         * THRESHOLD_INDEXER >= distanceIndexer; periodic.intakeDetected =
         * distanceIntake != 0 && THRESHOLD_INTAKE >= distanceIntake;
         */

        periodic.ball1Detected = tof1.getRange() < THRESHOLD_TOF1 && tof1.getRange() != 0.0;
        // periodic.ball2Detected = tof2.getRange() <
        // Constants.SUPERSTRUCTURE_THRESHOLD_TOF2 && tof2.getRange() != 0.0;
        // periodic.ball3Detected = tof3.getRange() <
        // Constants.SUPERSTRUCTURE_THRESHOLD_TOF3 && tof3.getRange() != 0.0;
        periodic.ball4Detected = tof4.getRange() < THRESHOLD_TOF4 && tof4.getRange() != 0.0;
        periodic.ball5Detected = tof5.getRange() < THRESHOLD_TOF5 && tof5.getRange() != 0.0;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                switch (periodic.state) {
                case DISABLED:
                case INIT:
                    if (periodic.ball1Detected) {
                        periodic.state = SuperState.ONE_TO_THREE_BALLS;
                    }
                    break;

                case ONE_TO_THREE_BALLS:
                    if (!periodic.ball1Detected) {
                        periodic.state = SuperState.INIT;
                    }
                    if (periodic.ball4Detected) {
                        if (!indexBoolean.isStarted()) {
                            indexBoolean.start();
                        } else if (indexBoolean.getBoolean()) {
                            periodic.state = SuperState.FULL_SYSTEM;
                        }
                    } else {
                        // Invalidate if it isn't true
                        indexBoolean.stop();
                    }
                    break;

                case FOUR_BALLS:
                    if (!periodic.ball1Detected) {
                        periodic.state = SuperState.INIT;
                    }
                    if (periodic.ball5Detected) {
                        periodic.state = SuperState.FULL_SYSTEM;
                    }
                    break;

                case FULL_SYSTEM:
                    if (!periodic.ball5Detected) {
                        periodic.state = SuperState.FOUR_BALLS;
                    }
                    break;

                case SHOOT:
                    break;
                default:
                }
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        switch (periodic.state) {
        case INIT:
            periodic.deliveryWheelDemand = .25;
            periodic.indexTopBeltDemand = .75;
            break;
        case ONE_TO_THREE_BALLS:
        case FOUR_BALLS:
            periodic.deliveryWheelDemand = periodic.indexTopBeltDemand = Constants.SUPER_DEMAND_STOP;
            break;
        case FULL_SYSTEM:
            periodic.intakeWheelsDemand = periodic.deliveryWheelDemand = periodic.indexTopBeltDemand = Constants.SUPER_DEMAND_STOP;
            break;
        case SHOOT:
            periodic.deliveryWheelDemand = 1;
            periodic.indexTopBeltDemand = .75;
            periodic.intakeWheelsDemand = 1;
            break;
        case DUMP_SYSTEM:
            periodic.deliveryWheelDemand = periodic.indexTopBeltDemand = -Constants.FULL_BELT_DEMAND;
            periodic.intakeWheelsDemand = -Constants.INTAKE_DEMAND;
            break;
        default:
        }

        deliveryWheel.set(ControlMode.PercentOutput, periodic.deliveryWheelDemand);
        indexTopBelt.set(ControlMode.PercentOutput, periodic.indexTopBeltDemand);
        intakeWheels.set(ControlMode.PercentOutput, periodic.intakeWheelsDemand);
        extensionArm.set(periodic.armExtension);
        periodic.currState = periodic.state.ordinal();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure/STATE", periodic.state.toString());
        SmartDashboard.putNumber("Superstructure/DELIVERY_WHEELS_DEMAND", periodic.deliveryWheelDemand);
        SmartDashboard.putNumber("Superstructure/INDEX_DEMAND", periodic.indexTopBeltDemand);
        SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.intakeWheelsDemand);
        SmartDashboard.putBoolean("Superstructure/BALL1", periodic.ball1Detected);
        // SmartDashboard.putBoolean("Superstructure/BALL3", periodic.ball3Detected);
        SmartDashboard.putBoolean("Superstructure/BALL4", periodic.ball4Detected);
        SmartDashboard.putBoolean("Superstructure/BALL5", periodic.ball5Detected);
        if (Constants.DEBUG) {
            /*
             * SmartDashboard.putNumber("Superstructure/Delivery_TOF_RAW",
             * deliverySensor.getRange());
             * SmartDashboard.putNumber("Superstructure/Index_TOF_RAW",
             * indexSensor.getRange());
             * SmartDashboard.putNumber("Superstructure/Intake_TOF_RAW",
             * intakeSensor.getRange());
             */
            SmartDashboard.putNumber("Superstructure/INDEX_AMPS", periodic.indexBeltAmps);
        }

    }

    public void shootBall() {
        if (!(periodic.state == SuperState.SHOOT)) {
            periodic.state = SuperState.SHOOT;
        }
    }

    public SuperState getState() {
        return periodic.state;
    }

    public void dumpSystem() {
        periodic.state = SuperState.DUMP_SYSTEM;
    }

    public void initState() {
        periodic.state = SuperState.INIT;
    }

    public void disable()
    {
        periodic.state = SuperState.DISABLED;
    }

    /**
     * Reset the values of the sensors, SRX, and reinitialize the IO.
     */
    @Override
    public void reset() {
        periodic = new SuperIO();

        deliveryWheel.setInverted(true);
        indexTopBelt.setInverted(false);
        intakeWheels.setInverted(false);

        tof1.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        tof2.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        tof3.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        tof4.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        tof5.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
    }

    // Setters
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? Value.kForward : Value.kReverse;
    }

    public void setIndexBeltDemand(double indexBeltDemand) {
        periodic.indexTopBeltDemand = indexBeltDemand;
    }

    public void setDeliveryWheelDemand(double demand) {
        periodic.deliveryWheelDemand = demand;
    }

    public void setIntakeDemand(double demand) {
        if (periodic.state == SuperState.DISABLED)
            periodic.state = SuperState.INIT;
        periodic.intakeWheelsDemand = demand;
    }

    // Getters
    public double getIndexBeltDemand() {
        return periodic.indexTopBeltDemand;
    }

    public boolean intakeDown() {
        return extensionArm.get() == Value.kForward;
    }

    public LogData getLogger() {
        return periodic;
    }

    public class SuperIO extends Subsystem.PeriodicIO {
        // Current State
        private SuperState state = SuperState.DISABLED;
        public int currState = state.ordinal();
        // Indexer Data
        public double indexBeltAmps = 0.0;
        public double deliveryWheelDemand;
        public double indexTopBeltDemand;
        // Intake Data
        public double intakeWheelsDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;
        // TOF Booleans
        public boolean ball1Detected;
        // public boolean ball2Detected;
        // public boolean ball3Detected;
        public boolean ball4Detected;
        public boolean ball5Detected;
    }

    public enum SuperState {
        INIT, ONE_TO_THREE_BALLS, FOUR_BALLS, FULL_SYSTEM, SHOOT, DUMP_SYSTEM, DISABLED;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}
