package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.drivers.SimTimeOfFlight;
import frc.robot.Constants;

import static frc.robot.Constants.*;

/**
 * Combines both the indexer and intake functions for performance economy.
 */
public class Superstructure extends Subsystem {
    // Global
    private SuperIO periodic;

    // Indexer
    private TalonSRX deliveryBelts;
    private TalonSRX indexBelt;

    // Intake
    private DoubleSolenoid extensionArm;
    private TalonSRX ballsIntake;

    // Sensors
    private SimTimeOfFlight deliverySensor;
    private SimTimeOfFlight indexSensor;
    private SimTimeOfFlight intakeSensor;

    private static Superstructure instance = new Superstructure();
    public static Superstructure getInstance() {
        return instance;
    }

    private Superstructure() {
        indexBelt = new TalonSRX(SUPERSTRUCTURE_INDEX_BELT);
        deliveryBelts = new TalonSRX(SUPERSTRUCTURE_DELIVERY_BELT);

        extensionArm = new DoubleSolenoid(TRANS_LOW_ID, TRANS_HIGH_ID);
        ballsIntake = new TalonSRX(SUPERSTRUCTURE_INTAKE);

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
    }

    /**
     * Read data from the sensors
     */
    @Override public synchronized void readPeriodicInputs() {
        if (Constants.DEBUG) {
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
    @Override public synchronized void writePeriodicOutputs() {
        if (Constants.DEBUG) {
            deliveryBelts.set(ControlMode.PercentOutput, SmartDashboard.getNumber("DELIVERY_DEMAND", periodic.deliveryBeltsDemand));
            indexBelt.set(ControlMode.PercentOutput, SmartDashboard.getNumber("INDEXER_DEMAND", periodic.indexBeltDemand));
            ballsIntake.set(ControlMode.PercentOutput, SmartDashboard.getNumber("INTAKE_DEMAND", periodic.intakeDemand));
        } else {
            deliveryBelts.set(ControlMode.PercentOutput, periodic.deliveryBeltsDemand);
            indexBelt.set(ControlMode.PercentOutput, periodic.indexBeltDemand);
            ballsIntake.set(ControlMode.PercentOutput, periodic.intakeDemand);
            extensionArm.set(periodic.armExtension);
        }
    }

    @Override public void outputTelemetry() {
    }

    /**
     * Reset the values of the sensors, and reinitialize the IO.
     */
    @Override public void reset() {
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
        // Indexer Data
        public double indexBeltDemand;
        public double deliveryBeltsDemand;
        // Intake Data
        public int ballCount;
        public double intakeDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kOff;
        // Sensor Data
        private double deliveryDistance;
        private double indexDistance;
        private double intakeDistance;
    }
}
