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
    private TimerBoolean ball4Timer;

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

    private double THRESHOLD_TOF1 = Constants.SUPERSTRUCTURE_THRESHOLD_TOF1;
    private double THRESHOLD_TOF2 = Constants.SUPERSTRUCTURE_THRESHOLD_TOF2;
    private double THRESHOLD_TOF3 = Constants.SUPERSTRUCTURE_THRESHOLD_TOF3;
    private double THRESHOLD_TOF4 = Constants.SUPERSTRUCTURE_THRESHOLD_TOF4;
    private double THRESHOLD_TOF5 = Constants.SUPERSTRUCTURE_THRESHOLD_TOF5;

    private Superstructure() {
        ball4Timer = new TimerBoolean(.25);
        deliveryWheel = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_WHEEL);
        deliveryBelts = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_BELT);
        indexTopBelt = new TalonSRX(Constants.SUPERSTRUCTURE_INDEX_BELT);

        intakeWheels = new TalonSRX(Constants.SUPERSTRUCTURE_INTAKE);
        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        TOF1 = new SimTimeOfFlight(Constants.SUPERSTURCTURE_TOF1_ID);
        TOF2 = new SimTimeOfFlight(Constants.SUPERSTURCTURE_TOF2_ID);
        TOF3 = new SimTimeOfFlight(Constants.SUPERSTURCTURE_TOF3_ID);
        TOF4 = new SimTimeOfFlight(Constants.SUPERSTURCTURE_TOF4_ID);
        TOF5 = new SimTimeOfFlight(Constants.SUPERSTURCTURE_TOF5_ID);

        reset();
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/TOF_1_DISTANCE", TOF1.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_2_DISTANCE", TOF2.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_3_DISTANCE", TOF3.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_4_DISTANCE", TOF4.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_5_DISTANCE", TOF5.getRange());
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
            SmartDashboard.putNumber("Superstructure/TOF_1_DISTANCE", TOF1.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_2_DISTANCE", TOF2.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_3_DISTANCE", TOF3.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_4_DISTANCE", TOF4.getRange());
            SmartDashboard.putNumber("Superstructure/TOF_5_DISTANCE", TOF5.getRange());
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

        periodic.BALL1Detected = TOF1.getRange() < Constants.SUPERSTRUCTURE_THRESHOLD_TOF1 && TOF1.getRange() != 0.0;
        periodic.BALL2Detected = TOF2.getRange() < Constants.SUPERSTRUCTURE_THRESHOLD_TOF2 && TOF2.getRange() != 0.0;
        periodic.BALL3Detected = TOF3.getRange() < Constants.SUPERSTRUCTURE_THRESHOLD_TOF3 && TOF3.getRange() != 0.0;
        periodic.BALL4Detected = TOF4.getRange() < Constants.SUPERSTRUCTURE_THRESHOLD_TOF4 && TOF4.getRange() != 0.0;
        periodic.BALL5Detected = TOF5.getRange() < Constants.SUPERSTRUCTURE_THRESHOLD_TOF5 && TOF5.getRange() != 0.0;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {

        enabledLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {
            }

            @Override
            public void onLoop(double timestamp) {

            }

            @Override
            public void onStop(double timestamp) {
            }
        });
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (periodic.state == SuperState.INTAKE) {
            periodic.intakeWheelsDemand = Constants.FULL_BELT_DEMAND;
        } else if (periodic.state == SuperState.SHOOT) {
            periodic.intakeWheelsDemand = periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand = Constants.FULL_BELT_DEMAND;
        } else if (periodic.state == SuperState.DUMP_SYSTEM) {
            periodic.intakeWheelsDemand = periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand = -Constants.FULL_BELT_DEMAND;
        } else {
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
        SmartDashboard.putNumber("Superstructure/DELIVERY_WHEELS_DEMAND", periodic.deliveryWheelDemand);
        SmartDashboard.putNumber("Superstructure/INDEX_DEMAND", periodic.indexTopBeltDemand);
        SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.intakeWheelsDemand);
        SmartDashboard.putBoolean("Superstructure/BALL1", periodic.BALL1Detected);
        SmartDashboard.putBoolean("Superstructure/BALL2", periodic.BALL2Detected);
        SmartDashboard.putBoolean("Superstructure/BALL3", periodic.BALL3Detected);
        SmartDashboard.putBoolean("Superstructure/BALL4", periodic.BALL4Detected);
        SmartDashboard.putBoolean("Superstructure/BALL5", periodic.BALL5Detected);
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

    public void intakeState() {
        periodic.state = SuperState.INTAKE;
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
        return periodic.BALL1Detected;
    }

    public boolean BALL2Detected() {
        return periodic.BALL2Detected;
    }

    public boolean BALL3Detected() {
        return periodic.BALL3Detected;
    }

    public boolean BALL4Detected() {
        if (periodic.BALL4Detected) {
            if (!ball4Timer.isStarted()) {
                ball4Timer.start();
            } else if (ball4Timer.getBoolean()) {
                return true;
            }
        }
        ball4Timer.stop();
        return false;
    }

    public boolean BALL5Detected() {
        return periodic.BALL5Detected;
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
        public boolean BALL1Detected;
        public boolean BALL2Detected;
        public boolean BALL3Detected;
        public boolean BALL4Detected;
        public boolean BALL5Detected;
    }

    public enum SuperState {
        INIT, INTAKE, SHOOT, DUMP_SYSTEM, DISABLED;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}
