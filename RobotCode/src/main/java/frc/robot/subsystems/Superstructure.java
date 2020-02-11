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
    private double DISTANCE_DELIVERY = 25.4;
    private double DISTANCE_INDEXER = 25.4;
    private double DISTANCE_INTAKE = 25.4;

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
        SmartDashboard.putNumber("DELIVERY_SENSOR_DISTANCE", periodic.deliveryDistance);
        SmartDashboard.putNumber("INDEXER_SENSOR_DISTANCE", periodic.indexDistance);
        SmartDashboard.putNumber("INTAKE_SENSOR_DISTANCE", periodic.intakeDistance);
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

            periodic.deliveryDistance = SmartDashboard.getNumber("DELIVERY_SENSOR_DISTANCE", periodic.deliveryDistance);
            periodic.indexDistance = SmartDashboard.getNumber("INDEXER_SENSOR_DISTANCE", periodic.indexDistance);
            periodic.intakeDistance = SmartDashboard.getNumber("INTAKE_SENSOR_DISTANCE", periodic.intakeDistance);
        } else {
            periodic.deliveryDistance = deliverySensor.getRange();
            periodic.indexDistance = indexSensor.getRange();
            periodic.intakeDistance = intakeSensor.getRange();
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
                    if (periodic.deliveryDistance < 20) { /* C2 */
                        periodic.state = SuperState.ONE_TO_FOUR_BALLS;
                    }
                    break;
                case ONE_TO_FOUR_BALLS:
                    oneToFourBalls();
                    if (periodic.indexDistance < 20 && periodic.deliveryDistance < 20) { /* C4 */
                        periodic.state = SuperState.FULL_SYSTEM;
                    }
                    break;
                case FULL_SYSTEM:
                    fullSystem();
                    break;
                case SHOOT:
                    shoot();
                    if (!(periodic.indexDistance < 20) && !(periodic.deliveryDistance < 20)) { /* C9 */
                        periodic.state = SuperState.INIT;
                    } else if (periodic.deliveryDistance < 20 && periodic.indexDistance < 20) { /* C7 */
                        periodic.state = SuperState.ONE_TO_FOUR_BALLS;
                    }
                    break;
                case DUMP_SYSTEM:
                    dumpSystem();
                    break;
                default:
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

    public void oneBall() {
        shooterWheel.set(ControlMode.PercentOutput, 0);
        deliveryBelts.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
    }

    public void oneToFourBalls() {
        shooterWheel.set(ControlMode.PercentOutput, 0);
        deliveryBelts.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, 0);
        ballsIntake.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
    }

    public void fullSystem() {
        shooterWheel.set(ControlMode.PercentOutput, 0);
        deliveryBelts.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, 0);
    }

    public void shoot() {
        shooterWheel.set(ControlMode.PercentOutput, FULL_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, STOP_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, 0);
    }

    public void dumpSystem() {
        shooterWheel.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        deliveryBelts.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        indexBelt.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
        ballsIntake.set(ControlMode.PercentOutput, -FULL_BELT_DEMAND);
    }

    public void shootBalls() {
        periodic.state = SuperState.SHOOT;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("DELIVERY_SENSOR_BOOL", DISTANCE_DELIVERY >= getDeliveryDistance());
        SmartDashboard.putBoolean("INDEXER_SENSOR_BOOL", DISTANCE_INDEXER >= getIndexDistance());
        SmartDashboard.putBoolean("INTAKE_SENSOR_BOOL", DISTANCE_INTAKE >= getIntakeDistance());
    }

    public void setShoot() {
        if (periodic.state == SuperState.ONE_TO_FOUR_BALLS || periodic.state == SuperState.ONE_BALL)
            periodic.state = SuperState.SHOOT;
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

    public double getDeliveryDistance() {
        return periodic.deliveryDistance;
    }

    public double getIndexDistance() {
        return periodic.indexDistance;
    }

    public double getIntakeDistance() {
        return periodic.intakeDistance;
    }

    public LogData getLogger() {
        return periodic;
    }

    public class SuperIO extends Subsystem.PeriodicIO {
        // Current State
        public SuperState state = SuperState.ONE_BALL;
        // Indexer Data
        public double indexBeltDemand;
        public double deliveryBeltsDemand;
        // Intake Data
        public int ballCount;
        public double intakeDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kOff;
        // Sensor Data
        public double deliveryDistance;
        public double indexDistance;
        public double intakeDistance;
    }

    enum SuperState {
        INIT(0), ONE_BALL(1), ONE_TO_FOUR_BALLS(2), FULL_SYSTEM(3), SHOOT(4), DUMP_SYSTEM(5);

        private int stateNumber;

        SuperState(int stateNumber) {
            this.stateNumber = stateNumber;
        }

    }
}
