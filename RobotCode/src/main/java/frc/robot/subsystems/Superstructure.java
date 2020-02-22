package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
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
    private SimTimeOfFlight deliverySensor;
    private SimTimeOfFlight indexSensor;
    private SimTimeOfFlight intakeSensor;

    // Double
    private double THRESHOLD_DELIVERY = Constants.THRESHOLD_DELIVERY;
    private double THRESHOLD_INDEXER = Constants.THRESHOLD_INDEXER;
    private double THRESHOLD_INTAKE = Constants.THRESHOLD_INTAKE;
    private double distanceDelivery;
    private double distanceIndexer;
    private double distanceIntake;

    // TimedBooleans
    private TimerBoolean indexBoolean = new TimerBoolean(Constants.TIME_TILL_STATIONARY);

    // Pulse Variables
    private double pulseTimestamp;
    private double pulseCooldownTimestamp;

    private static Superstructure instance = new Superstructure();

    public static Superstructure getInstance() {
        return instance;
    }

    private Superstructure() {
        deliveryWheel = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_WHEEL);
        deliveryBelts = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_BELT);
        indexTopBelt = new TalonSRX(Constants.SUPERSTRUCTURE_INDEX_BELT);

        intakeWheels = new TalonSRX(Constants.SUPERSTRUCTURE_INTAKE);
        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        deliverySensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_DELIVERY);
        indexSensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_INDEX);
        intakeSensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_INTAKE);

        reset();
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/DELIVERY_SENSOR_THRESHOLD", THRESHOLD_DELIVERY);
            SmartDashboard.putNumber("Superstructure/INDEXER_SENSOR_THRESHOLD", THRESHOLD_INDEXER);
            SmartDashboard.putNumber("Superstructure/INTAKE_SENSOR_THRESHOLD", THRESHOLD_INTAKE);
            SmartDashboard.putNumber("Superstructure/DELIVERY_SENSOR_DISTANCE", deliverySensor.getRange());
            SmartDashboard.putNumber("Superstructure/INDEXER_SENSOR_DISTANCE", indexSensor.getRange());
            SmartDashboard.putNumber("Superstructure/INTAKE_SENSOR_DISTANCE", intakeSensor.getRange());
        }
    }

    /**
     * Read data from the sensors
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
            THRESHOLD_DELIVERY = SmartDashboard.getNumber("Superstructure/DELIVERY_SENSOR_THRESHOLD",
                    THRESHOLD_DELIVERY);
            THRESHOLD_INDEXER = SmartDashboard.getNumber("Superstructure/INDEXER_SENSOR_THRESHOLD", THRESHOLD_INDEXER);
            THRESHOLD_INTAKE = SmartDashboard.getNumber("Superstructure/INTAKE_SENSOR_THRESHOLD", THRESHOLD_INTAKE);

            distanceDelivery = deliverySensor.getRange();
            distanceIndexer = indexSensor.getRange();
            distanceIntake = intakeSensor.getRange();
        } else {
            distanceDelivery = deliverySensor.getRange();
            distanceIndexer = indexSensor.getRange();
            distanceIntake = intakeSensor.getRange();
        }
        periodic.indexBeltAmps = indexTopBelt.getSupplyCurrent();
        periodic.deliveryDetected = distanceDelivery != 0 && THRESHOLD_DELIVERY >= distanceDelivery;
        periodic.indexDetected = distanceIndexer != 0 && THRESHOLD_INDEXER >= distanceIndexer;
        periodic.intakeDetected = distanceIntake != 0 && THRESHOLD_INTAKE >= distanceIntake;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        switch (periodic.state) {
        case INIT:
            periodic.deliveryWheelDemand = periodic.indexTopBeltDemand = Constants.FULL_BELT_DEMAND;
            periodic.deliveryBeltsDemand = Constants.STOP_BELT_DEMAND;
            break;
        case ONE_TO_THREE_BALLS:
            pulseIndexDemand();
            periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = Constants.STOP_BELT_DEMAND;
            break;
        case FOUR_BALLS:
            periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND;
            break;
        case FULL_SYSTEM:
            periodic.intakeWheelsDemand = periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND;
            break;
        case SHOOT:
            periodic.deliveryBeltsDemand = .5;
            periodic.deliveryWheelDemand = Constants.FULL_BELT_DEMAND;
            periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND;
            break;
        case DUMP_SYSTEM:
            periodic.deliveryWheelDemand = periodic.deliveryBeltsDemand = periodic.indexTopBeltDemand = -1 * Constants.FULL_BELT_DEMAND;
            periodic.intakeWheelsDemand = -1 * Constants.INTAKE_DEMAND; 
            break;
        default:

        }

        deliveryBelts.set(ControlMode.PercentOutput, periodic.deliveryBeltsDemand);
        deliveryWheel.set(ControlMode.PercentOutput, periodic.deliveryWheelDemand);
        indexTopBelt.set(ControlMode.PercentOutput, periodic.indexTopBeltDemand);
        intakeWheels.set(ControlMode.PercentOutput, periodic.intakeWheelsDemand);
        extensionArm.set(periodic.armExtension);
        periodic.currState = periodic.state.ordinal();
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
                    if (periodic.deliveryDetected) {
                        periodic.state = SuperState.ONE_TO_THREE_BALLS;
                    }
                    break;

                case ONE_TO_THREE_BALLS:
                    if (periodic.indexDetected) {
                        if (!indexBoolean.isStarted()) {
                            indexBoolean.start();
                        } else if (indexBoolean.getBoolean()) {
                            periodic.state = SuperState.FOUR_BALLS;
                        }
                    } else {
                        // Invalidate if it isn't true
                        indexBoolean.stop();
                    }
                    break;

                case FOUR_BALLS:
                    if (periodic.intakeDetected) {
                        periodic.state = SuperState.FULL_SYSTEM;
                    }
                    break;

                case FULL_SYSTEM:

                    if (!periodic.intakeDetected) {
                        periodic.state = SuperState.FOUR_BALLS;
                    }

                    break;

                case SHOOT:
                    if (!periodic.deliveryDetected) {
                        periodic.state = SuperState.INIT;
                    }
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
    public void outputTelemetry() {
        SmartDashboard.putString("Superstructure/STATE", periodic.state.toString());
        SmartDashboard.putBoolean("Superstructure/Delivery_TOF", deliveryDetected());
        SmartDashboard.putBoolean("Superstructure/Index_TOF", indexDetected());
        SmartDashboard.putBoolean("Superstructure/Intake_TOF", intakeDetected());
        SmartDashboard.putNumber("Superstructure/DELIVERY_DEMAND", periodic.deliveryBeltsDemand);
        SmartDashboard.putNumber("Superstructure/INDEX_DEMAND", periodic.indexTopBeltDemand);
        SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.intakeWheelsDemand);
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Superstructure/Delivery_TOF_RAW", deliverySensor.getRange());
            SmartDashboard.putNumber("Superstructure/Index_TOF_RAW", indexSensor.getRange());
            SmartDashboard.putNumber("Superstructure/Intake_TOF_RAW", intakeSensor.getRange());
        }
        SmartDashboard.putNumber("Superstructure/INDEX_AMPS", periodic.indexBeltAmps);
    }

    public void pulseIndexDemand() {
        if (pulseCooldownTimestamp <= Timer.getFPGATimestamp()) {
            pulseTimestamp = Timer.getFPGATimestamp() + Constants.PULSE_LENGTH;
            pulseCooldownTimestamp = pulseTimestamp + Constants.PULSE_COOLDOWN;
            periodic.indexTopBeltDemand = Constants.FULL_BELT_DEMAND;
        } else if (pulseTimestamp <= Timer.getFPGATimestamp()) {
            periodic.indexTopBeltDemand = Constants.STOP_BELT_DEMAND;
        }
    }

    public void shootBall() {
        if (periodic.state == SuperState.ONE_TO_THREE_BALLS || periodic.state == SuperState.FOUR_BALLS
                || periodic.state == SuperState.FULL_SYSTEM) {
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
        deliverySensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        indexSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        intakeSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
    }

    // Setters
    public void setArmExtension(boolean armExtension) {
        periodic.armExtension = armExtension? Value.kForward : Value.kReverse;
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
    public DoubleSolenoid.Value getArmExtension() {
        return periodic.armExtension;
    }

    public double getIndexBeltDemand() {
        return periodic.indexTopBeltDemand;
    }

    public LogData getLogger() {
        return periodic;
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

    public boolean intakeDown() {
        return extensionArm.get() == Value.kForward;
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
    }

    public enum SuperState {
        INIT, ONE_TO_THREE_BALLS, FOUR_BALLS, FULL_SYSTEM, SHOOT, DUMP_SYSTEM, DISABLED;

        public String toString() {
            return name().charAt(0) + name().substring(1).toLowerCase();
        }
    }
}
