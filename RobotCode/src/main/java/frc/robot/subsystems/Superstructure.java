package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.DemandUtil;
import frc.lib.util.TimerBoolean;
import frc.robot.Constants;

import static frc.robot.Constants.TIME_TILL_STATIONARY;

/**
 * Combines both the indexer and intake functions for performance economy.
 */
public class Superstructure extends Subsystem {
    // Global
    private SuperIO periodic;

    // Indexer
    private TalonSRX shooterWheel;
    private TalonSRX deliveryBelts;
    private TalonSRX indexBelt;

    // Intake
    private TalonSRX ballsIntake;
    private DoubleSolenoid extensionArm;

    // Sensors
    private SimTimeOfFlight deliverySensor;
    private SimTimeOfFlight indexSensor;
    private SimTimeOfFlight intakeSensor;

    // Bool
    private double DISTANCE_DELIVERY = 75;
    private double DISTANCE_INDEXER = 75;
    private double DISTANCE_INTAKE = 75;

    // TimedBooleans
    private TimerBoolean indexBoolean = new TimerBoolean(TIME_TILL_STATIONARY);
    private TimerBoolean intakeBoolean = new TimerBoolean(TIME_TILL_STATIONARY);

    private static Superstructure instance = new Superstructure();

    public static Superstructure getInstance() {
        return instance;
    }

    private Superstructure() {
        shooterWheel = new TalonSRX(Constants.SUPERSTRUCTURE_SHOOTER_WHEEL);
        deliveryBelts = new TalonSRX(Constants.SUPERSTRUCTURE_DELIVERY_BELT);
        indexBelt = new TalonSRX(Constants.SUPERSTRUCTURE_INDEX_BELT);

        ballsIntake = new TalonSRX(Constants.SUPERSTRUCTURE_INTAKE);
        extensionArm = new DoubleSolenoid(Constants.INTAKE_HIGH_ID, Constants.INTAKE_LOW_ID);

        deliverySensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_DELIVERY);
        indexSensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_INDEX);
        intakeSensor = new SimTimeOfFlight(Constants.FLIGHT_SENSOR_INTAKE);

        reset();

        SmartDashboard.putNumber("Superstructure/BALLS", periodic.ballCount);
        SmartDashboard.putNumber("Superstructure/DELIVERY_DEMAND", periodic.deliveryBeltsDemand);
        SmartDashboard.putNumber("Superstructure/INDEXER_DEMAND", periodic.indexBeltDemand);
        SmartDashboard.putNumber("Superstructure/INTAKE_DEMAND", periodic.intakeDemand);
        SmartDashboard.putNumber("Superstructure/DELIVERY_SENSOR_DISTANCE", deliverySensor.getRange());
        SmartDashboard.putNumber("Superstructure/INDEXER_SENSOR_DISTANCE", indexSensor.getRange());
        SmartDashboard.putNumber("Superstructure/INTAKE_SENSOR_DISTANCE", intakeSensor.getRange());
    }

    /**
     * Read data from the sensors
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
            DISTANCE_DELIVERY = SmartDashboard.getNumber("Superstructure/DELIVERY_SENSOR_THRESHOLD", 0);
            DISTANCE_INDEXER = SmartDashboard.getNumber("Superstructure/INDEXER_SENSOR_THRESHOLD", 0);
            DISTANCE_INTAKE = SmartDashboard.getNumber("Superstructure/INTAKE_SENSOR_THRESHOLD", 0);

            periodic.deliveryDetected = SmartDashboard.getBoolean("Superstructure/DELIVERY_SENSOR_DISTANCE",
                    periodic.deliveryDetected);
            periodic.indexDetected = SmartDashboard.getBoolean("Superstructure/INDEXER_SENSOR_DISTANCE",
                    periodic.indexDetected);
            periodic.intakeDetected = SmartDashboard.getBoolean("Superstructure/INTAKE_SENSOR_DISTANCE",
                    periodic.intakeDetected);
        } else {
            periodic.deliveryDetected = deliverySensor.getRange() != 0 && DISTANCE_DELIVERY >= deliverySensor.getRange();
            periodic.indexDetected = indexSensor.getRange() != 0 && DISTANCE_INDEXER >= indexSensor.getRange();
            periodic.intakeDetected = intakeSensor.getRange() != 0 && DISTANCE_INTAKE >= intakeSensor.getRange();
        }
    }

    @Override public synchronized void writePeriodicOutputs() {
        switch (periodic.state) {
            case INIT:
                DemandUtil.setFullDemand(shooterWheel, deliveryBelts, indexBelt, ballsIntake);
                break;
            case ONE_TO_THREE_BALLS:
                DemandUtil.setFullDemand(deliveryBelts, ballsIntake);
                DemandUtil.disable(shooterWheel, indexBelt);
                break;
            case FOUR_BALLS:
                DemandUtil.setFullDemand(ballsIntake);
                DemandUtil.disable(shooterWheel, deliveryBelts, indexBelt);
                break;
            case FULL_SYSTEM:
                DemandUtil.disable(shooterWheel, deliveryBelts, indexBelt, ballsIntake);
                break;
            case SHOOT:
                DemandUtil.setFullDemand(shooterWheel);
                DemandUtil.disable(deliveryBelts, indexBelt, ballsIntake);
                break;
            case DUMP_SYSTEM:
                dumpSystem();
                break;
            default:
                DemandUtil.setFullBackDemand(shooterWheel, deliveryBelts, indexBelt, ballsIntake);
                break;
        }
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
                    case INIT: {
                        if (periodic.deliveryDetected) {
                            periodic.state = SuperState.ONE_TO_THREE_BALLS;
                        }
                        break;
                    }
                    case ONE_TO_THREE_BALLS: {
                        if (periodic.indexDetected) {
                            if (indexBoolean.isStarted()) {
                                indexBoolean.start();
                            } else if (indexBoolean.getBoolean()) {
                                periodic.state = SuperState.FOUR_BALLS;
                            }
                        } else {
                            // Invalidate if it isn't true
                            indexBoolean.stop();
                        }
                        break;
                    }
                    case FOUR_BALLS: {
                        if (periodic.intakeDetected) {
                            if (intakeBoolean.isStarted()) {
                                intakeBoolean.start();
                            } else if (intakeBoolean.getBoolean()) {
                                periodic.state = SuperState.FULL_SYSTEM;
                            }
                        } else {
                            // Invalidate if it isn't true
                            intakeBoolean.stop();
                        }
                        break;
                    }
                    case SHOOT: {
                        if (!periodic.deliveryDetected) {
                            periodic.state = SuperState.INIT;
                        }
                        break;
                    }
                    case DUMP_SYSTEM: case FULL_SYSTEM: default: break;
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Superstructure/Delivery_TOF", deliveryDetected());
        SmartDashboard.putBoolean("Superstructure/Index_TOF", indexDetected());
        SmartDashboard.putBoolean("Superstructure/Intake_TOF", intakeDetected());
        SmartDashboard.putNumber("Superstructure/Delivery_TOF_RAW", deliverySensor.getRange());
        SmartDashboard.putNumber("Superstructure/Index_TOF_RAW", indexSensor.getRange());
        SmartDashboard.putNumber("Superstructure/Intake_TOF_RAW",  intakeSensor.getRange());
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
     * Reset the values of the sensors, and reinitialize the IO.
     */
    @Override
    public void reset() {
        periodic = new SuperIO();

        deliverySensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        indexSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        intakeSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
    }

    // Setters
    public void setBallCount(int ballCount) {
        periodic.ballCount = ballCount;
    }

    public void setArmExtension(DoubleSolenoid.Value armExtension) {
        periodic.armExtension = armExtension;
    }

    public void setIndexBeltDemand(double indexBeltDemand) {
        periodic.indexBeltDemand = indexBeltDemand;
    }

    public void setDeliveryBeltsDemand(double demand) {
        periodic.deliveryBeltsDemand = demand;
    }

    public void setIntakeDemand(double demand) {
        periodic.intakeDemand = demand;
    }

    // Getters
    public int getBallCount() {
        return periodic.ballCount;
    }

    public DoubleSolenoid.Value getArmExtension() {
        return periodic.armExtension;
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

    public LogData getLogger() {
        return periodic;
    }

    public class SuperIO extends Subsystem.PeriodicIO {
        // Current State
        public SuperState state = SuperState.INIT;
        // Indexer Data
        public double indexBeltDemand;
        public double deliveryBeltsDemand;
        // Intake Data
        public int ballCount;
        public double intakeDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kOff;
        // Sensor Booleans
        public boolean deliveryDetected;
        public boolean indexDetected;
        public boolean intakeDetected;
    }

    public enum SuperState {
        INIT(0), ONE_TO_THREE_BALLS(1), FOUR_BALLS(2), FULL_SYSTEM(3), SHOOT(4), DUMP_SYSTEM(5);

        private int stateNumber;

        SuperState(int stateNumber) {
            this.stateNumber = stateNumber;
        }

        public int getStateNumber() {
            return stateNumber;
        }
    }
}
