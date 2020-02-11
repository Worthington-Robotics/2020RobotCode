package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.TimedBoolean;
import frc.robot.Constants;

import static frc.robot.Constants.*;

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
    private double DISTANCE_DELIVERY = 20;
    private double DISTANCE_INDEXER = 20;
    private double DISTANCE_INTAKE = 20;

    // TimedBooleans
    private TimedBoolean indexBoolean;
    private TimedBoolean intakeBoolean;

    private static Superstructure instance = new Superstructure();

    public static Superstructure getInstance() {
        return instance;
    }

    private Superstructure() {
        shooterWheel = new TalonSRX(SUPERSTRUCTURE_SHOOTER_WHEEL);
        deliveryBelts = new TalonSRX(SUPERSTRUCTURE_DELIVERY_BELT);
        indexBelt = new TalonSRX(SUPERSTRUCTURE_INDEX_BELT);

        ballsIntake = new TalonSRX(SUPERSTRUCTURE_INTAKE);
        extensionArm = new DoubleSolenoid(TRANS_LOW_ID, TRANS_HIGH_ID);

        deliverySensor = new SimTimeOfFlight(FLIGHT_SENSOR_DELIVERY);
        indexSensor = new SimTimeOfFlight(FLIGHT_SENSOR_INDEX);
        intakeSensor = new SimTimeOfFlight(FLIGHT_SENSOR_INTAKE);

        reset();

        SmartDashboard.putNumber("BALLS", periodic.ballCount);
        SmartDashboard.putNumber("DELIVERY_DEMAND", periodic.deliveryBeltsDemand);
        SmartDashboard.putNumber("INDEXER_DEMAND", periodic.indexBeltDemand);
        SmartDashboard.putNumber("INTAKE_DEMAND", periodic.intakeDemand);
        SmartDashboard.putNumber("DELIVERY_SENSOR_DISTANCE", deliverySensor.getRange());
        SmartDashboard.putNumber("INDEXER_SENSOR_DISTANCE", indexSensor.getRange());
        SmartDashboard.putNumber("INTAKE_SENSOR_DISTANCE", intakeSensor.getRange());
        SmartDashboard.putNumber("DELIVERY_SENSOR_THRESHOLD", DISTANCE_DELIVERY);
        SmartDashboard.putNumber("INDEXER_SENSOR_THRESHOLD", DISTANCE_INDEXER);
        SmartDashboard.putNumber("INTAKE_SENSOR_THRESHOLD", DISTANCE_INTAKE);
    }

    /**
     * Read data from the sensors
     */
    @Override
    public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
            DISTANCE_DELIVERY = SmartDashboard.getNumber("DELIVERY_SENSOR_THRESHOLD", 0);
            DISTANCE_INDEXER = SmartDashboard.getNumber("INDEXER_SENSOR_THRESHOLD", 0);
            DISTANCE_INTAKE = SmartDashboard.getNumber("INTAKE_SENSOR_THRESHOLD", 0);

            periodic.deliveryDetected = SmartDashboard.getBoolean("DELIVERY_SENSOR_DISTANCE", periodic.deliveryDetected);
            periodic.indexDetected = SmartDashboard.getBoolean("INDEXER_SENSOR_DISTANCE", periodic.indexDetected);
            periodic.intakeDetected = SmartDashboard.getBoolean("INTAKE_SENSOR_DISTANCE", periodic.intakeDetected);
        } else {
            periodic.deliveryDetected = DISTANCE_DELIVERY >= deliverySensor.getRange();
            periodic.indexDetected = DISTANCE_INDEXER >= indexSensor.getRange();
            periodic.intakeDetected = DISTANCE_INTAKE >= intakeSensor.getRange();
        }
    }

    /**
     * Update values of the SRXs, DoubleSolenoid
     */
    @Override
    public synchronized void writePeriodicOutputs() {
        /*
         * if (Constants.DEBUG) { deliveryBelts.set(ControlMode.PercentOutput,
         * SmartDashboard.getNumber("DELIVERY_DEMAND", periodic.deliveryBeltsDemand));
         * indexBelt.set(ControlMode.PercentOutput,
         * SmartDashboard.getNumber("INDEXER_DEMAND", periodic.indexBeltDemand));
         * ballsIntake.set(ControlMode.PercentOutput,
         * SmartDashboard.getNumber("INTAKE_DEMAND", periodic.intakeDemand)); } else {
         * deliveryBelts.set(ControlMode.PercentOutput, periodic.deliveryBeltsDemand);
         * indexBelt.set(ControlMode.PercentOutput, periodic.indexBeltDemand);
         * ballsIntake.set(ControlMode.PercentOutput, periodic.intakeDemand);
         * extensionArm.set(periodic.armExtension);
         */
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
                    case INIT:
                        init();
                        if (periodic.deliveryDetected) { /* C2 */
                            periodic.state = SuperState.ONE_TO_FOUR_BALLS;
                        }
                        break;
                    case ONE_TO_FOUR_BALLS:
                        oneToFourBalls();
                        if (periodic.indexDetected/* && periodic.deliveryDetected*/) { /* C4 */
                            if (indexBoolean == null) {
                                indexBoolean = new TimedBoolean();
                            } else if (indexBoolean.getBoolean()) {
                                periodic.state = SuperState.FULL_SYSTEM;
                            }
                        } else {
                            // Invalidate if it isn't true
                            indexBoolean = null;
                        }
                        break;
                    case FULL_SYSTEM:
                        fullSystem();
                        break;
                    case SHOOT:
                        shoot();
                        if ((periodic.indexDetected || periodic.deliveryDetected)) { /* C9 */
                            periodic.state = SuperState.INIT;
                        } else if (periodic.deliveryDetected && periodic.indexDetected) { /* C7 */
                            periodic.state = SuperState.ONE_TO_FOUR_BALLS;
                        }
                        break;
                    case DUMP_SYSTEM:
                        dumpSystem();
                        break;
                    default:
                        break;
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public void init() {
        shooterWheel.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
    }

    public void oneToFourBalls() {
        shooterWheel.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
    }

    public void fullSystem() {
        shooterWheel.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
    }

    public void shoot() {
        shooterWheel.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
    }

    public void dumpSystem() {
        shooterWheel.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Superstructure/Delivery_TOF", deliveryDetected());
        SmartDashboard.putBoolean("Superstructure/Index_TOF", indexDetected());
        SmartDashboard.putBoolean("Superstructure/Intake_TOF", intakeDetected());
    }

    public void shootBalls() {
        periodic.state = SuperState.SHOOT;
    }

    public void setShoot() {
        if (periodic.state == SuperState.ONE_TO_FOUR_BALLS || periodic.state == SuperState.FULL_SYSTEM) {
            periodic.state = SuperState.SHOOT;
        }
    }

    public void setDump() {
        periodic.state = SuperState.DUMP_SYSTEM;
    }
    public void setInit() {
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
        INIT(0),
        ONE_TO_FOUR_BALLS(1),
        FULL_SYSTEM(2),
        SHOOT(3),
        DUMP_SYSTEM(4);

        private int stateNumber;

        SuperState(int stateNumber) {
            this.stateNumber = stateNumber;
        }

        public int getStateNumber() {
            return stateNumber;
        }
    }
}
