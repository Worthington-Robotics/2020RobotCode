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

    // Indexer
    private TalonSRX deliveryWheel;
    private TalonSRX deliveryBelts;
    private TalonSRX indexTopBelt;

    // Intake
    private TalonSRX intakeWheels;
    private DoubleSolenoid extensionArm;

    // Sensors
    private SimTimeOfFlight TOF1;
    private SimTimeOfFlight TOF2;
    private SimTimeOfFlight TOF3;
    private SimTimeOfFlight TOF4;
    private SimTimeOfFlight TOF5;

    // Double
    private double THRESHOLD_DELIVERY = Constants.THRESHOLD_DELIVERY;
    private double THRESHOLD_INDEXER = Constants.THRESHOLD_INDEXER;
    private double THRESHOLD_INTAKE = Constants.THRESHOLD_INTAKE;

    private double THRESHOLD_TOF1 = Constants.THRESHOLD_TOF1;
    private double THRESHOLD_TOF2 = Constants.THRESHOLD_TOF2;
    private double THRESHOLD_TOF3 = Constants.THRESHOLD_TOF3;
    private double THRESHOLD_TOF4 = Constants.THRESHOLD_TOF4;
    private double THRESHOLD_TOF5 = Constants.THRESHOLD_TOF5;

    // TimedBooleans
    private TimerBoolean indexBoolean = new TimerBoolean(Constants.TIME_TILL_STATIONARY);

    private Superstructure() {
        deliveryWheel = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_WHEEL);
        deliveryBelts = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_BELT);
        indexTopBelt = new TalonSRX(Constants.SUPERSTRUCTURE_INDEX_BELT);

        intakeWheels = new TalonSRX(Constants.SUPERSTRUCTURE_INTAKE);
        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        TOF1 = new SimTimeOfFlight(1);
        TOF2 = new SimTimeOfFlight(2);
        TOF3 = new SimTimeOfFlight(3);
        TOF4 = new SimTimeOfFlight(4);
        TOF5 = new SimTimeOfFlight(5);

        reset();
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/DELIVERY_SENSOR_THRESHOLD", THRESHOLD_DELIVERY);
            SmartDashboard.putNumber("Superstructure/INDEXER_SENSOR_THRESHOLD", THRESHOLD_INDEXER);
            SmartDashboard.putNumber("Superstructure/INTAKE_SENSOR_THRESHOLD", THRESHOLD_INTAKE);
            SmartDashboard.putNumber("Superstructure/DELIVERY_SENSOR_DISTANCE", TOF1.getRange());
            // SmartDashboard.putNumber("Superstructure/INDEXER_SENSOR_DISTANCE",
            // indexSensor.getRange());
            // SmartDashboard.putNumber("Superstructure/INTAKE_SENSOR_DISTANCE",
            // intakeSensor.getRange());
            SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF1", THRESHOLD_TOF1);
            SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF2", THRESHOLD_TOF2);
            SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF3", THRESHOLD_TOF3);
            SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF4", THRESHOLD_TOF4);
            SmartDashboard.getNumber("Superstructure/THRESHOLD_TOF5", THRESHOLD_TOF5);
        }
    }

    /**
     * Read data from the sensors
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
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

        periodic.TOF1Detected = TOF1.getRange() < Constants.THRESHOLD_TOF1;
        periodic.TOF2Detected = TOF2.getRange() < Constants.THRESHOLD_TOF2;
        periodic.TOF3Detected = TOF3.getRange() < Constants.THRESHOLD_TOF3;
        periodic.TOF4Detected = TOF4.getRange() < Constants.THRESHOLD_TOF4;
        periodic.TOF5Detected = TOF5.getRange() < Constants.THRESHOLD_TOF5;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {

        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {
                /*
                 * switch (periodic.state) { case DISABLED: case INIT: if
                 * (periodic.deliveryDetected) { periodic.state = SuperState.ONE_TO_THREE_BALLS;
                 * } break;
                 * 
                 * case ONE_TO_THREE_BALLS: if (periodic.indexDetected) { if
                 * (!indexBoolean.isStarted()) { indexBoolean.start(); } else if
                 * (indexBoolean.getBoolean()) { periodic.state = SuperState.FOUR_BALLS; } }
                 * else { // Invalidate if it isn't true indexBoolean.stop(); } break;
                 * 
                 * case FOUR_BALLS: if (periodic.intakeDetected) { periodic.state =
                 * SuperState.FULL_SYSTEM; } break;
                 * 
                 * case FULL_SYSTEM:
                 * 
                 * if (!periodic.intakeDetected) { periodic.state = SuperState.FOUR_BALLS; }
                 * 
                 * break;
                 * 
                 * case SHOOT: if (!periodic.deliveryDetected) { periodic.state =
                 * SuperState.INIT; } break; default: }
                 */
            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        /*
         * switch (periodic.state) { case INIT: periodic.deliveryWheelDemand =
         * periodic.indexTopBeltDemand = Constants.FULL_BELT_DEMAND;
         * periodic.deliveryBeltsDemand = Constants.STOP_BELT_DEMAND; break; case
         * ONE_TO_THREE_BALLS: periodic.deliveryWheelDemand =
         * periodic.indexTopBeltDemand = periodic.deliveryBeltsDemand =
         * Constants.STOP_BELT_DEMAND; break; case FOUR_BALLS:
         * periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand =
         * periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND; break; case
         * FULL_SYSTEM: periodic.intakeWheelsDemand = periodic.deliveryWheelDemand =
         * periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand =
         * Constants.STOP_BELT_DEMAND; break; case SHOOT: periodic.deliveryBeltsDemand =
         * .5; periodic.deliveryWheelDemand = Constants.FULL_BELT_DEMAND;
         * periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND; break; case
         * DUMP_SYSTEM: periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand =
         * periodic.indexTopBeltDemand = -1 * Constants.FULL_BELT_DEMAND;
         * periodic.intakeWheelsDemand = -1 * Constants.INTAKE_DEMAND; break; default:
         * 
         * }
         */
        if (periodic.state != SuperState.DUMP_SYSTEM && periodic.state != SuperState.SHOOT) {
            if (!BALL2Detected() && BALL5Detected()) {
                periodic.indexTopBeltDemand = Constants.FULL_BELT_DEMAND;
            } else {
                periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND;
            }
            if (BALL2Detected() && !BALL1Detected()) {
                periodic.deliveryWheelDemand = Constants.FULL_BELT_DEMAND;
            } else {
                periodic.deliveryWheelDemand = Constants.STOP_BELT_DEMAND;
            }
            if (BALL5Detected() && !BALL4Detected()) {
                periodic.intakeWheelsDemand = Constants.INTAKE_DEMAND;
            } else {
                periodic.intakeWheelsDemand = Constants.STOP_BELT_DEMAND;
            }
        }

        deliveryBelts.set(ControlMode.PercentOutput, periodic.deliveryBeltsDemand);
        deliveryWheel.set(ControlMode.PercentOutput, periodic.deliveryWheelDemand);
        indexTopBelt.set(ControlMode.PercentOutput, periodic.indexTopBeltDemand);
        intakeWheels.set(ControlMode.PercentOutput, periodic.intakeWheelsDemand);
        extensionArm.set(periodic.armExtension);
        periodic.currState = periodic.state.ordinal();
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure/STATE", periodic.state.toString());
        SmartDashboard.putBoolean("Superstructure/Delivery_TOF", deliveryDetected());
        SmartDashboard.putBoolean("Superstructure/Index_TOF", indexDetected());
        SmartDashboard.putBoolean("Superstructure/Intake_TOF", intakeDetected());
        SmartDashboard.putNumber("Superstructure/DELIVERY_DEMAND", periodic.deliveryBeltsDemand);
        SmartDashboard.putNumber("Superstructure/INDEX_DEMAND", periodic.indexTopBeltDemand);
        SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.intakeWheelsDemand);
        SmartDashboard.putBoolean("Superstructure/BALL1", BALL1Detected());
        SmartDashboard.putBoolean("Superstructure/BALL2", BALL2Detected());
        SmartDashboard.putBoolean("Superstructure/BALL3", BALL3Detected());
        SmartDashboard.putBoolean("Superstructure/BALL4", BALL4Detected());
        SmartDashboard.putBoolean("Superstructure/BALL5", BALL5Detected());
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

    public void dumpSystem() {
        periodic.state = SuperState.DUMP_SYSTEM;
    }

    public void initState() {
        periodic.state = SuperState.INIT;
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
        deliveryBelts.setInverted(false);

        /*
         * deliverySensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
         * indexSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
         * intakeSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
         */
        TOF1.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        TOF2.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        TOF3.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        TOF4.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        TOF5.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
    }

    // Setters
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension ? Value.kForward : Value.kReverse;
    }

    public void setIndexBeltDemand(double indexBeltDemand) {
        periodic.indexTopBeltDemand = indexBeltDemand;
    }

    public void setDeliveryBeltsDemand(double demand) {
        periodic.deliveryBeltsDemand = demand;
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

    public boolean deliveryDetected() {
        return periodic.deliveryDetected;
    }

    public boolean indexDetected() {
        return periodic.indexDetected;
    }

    public boolean intakeDetected() {
        return periodic.intakeDetected;
    }

    public boolean BALL1Detected() {
        return periodic.TOF1Detected;
    }

    public boolean BALL2Detected() {
        return periodic.TOF2Detected;
    }

    public boolean BALL3Detected() {
        return periodic.TOF3Detected;
    }

    public boolean BALL4Detected() {
        return periodic.TOF4Detected;
    }

    public boolean BALL5Detected() {
        return periodic.TOF5Detected;
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
        public double deliveryBeltsDemand;
        // Intake Data
        public double intakeWheelsDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kReverse;
        // Sensor Booleans
        public boolean deliveryDetected;
        public boolean indexDetected;
        public boolean intakeDetected;
        // TOF Booleans
        public boolean TOF1Detected;
        public boolean TOF2Detected;
        public boolean TOF3Detected;
        public boolean TOF4Detected;
        public boolean TOF5Detected;
    }

    public enum SuperState {
        INIT, INTAKE, SHOOT, DUMP_SYSTEM, DISABLED;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}
