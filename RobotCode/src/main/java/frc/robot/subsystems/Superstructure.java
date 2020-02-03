package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import static frc.robot.Constants.*;

/**
 * Combines both the indexer and intake functions for performance economy.
 */
public class Superstructure extends Subsystem {
    // Global
    private SuperIO periodic;

    // Indexer
    private TalonSRX indexBeltAbove;
    private TalonSRX indexBeltBelow;

    private TalonSRX deliveryBelt;

    // Intake
    private DoubleSolenoid extensionArm;
    private TalonSRX ballsIntake;

    // Sensors
    private TimeOfFlight deliverySensor;
    private TimeOfFlight indexSensor;
    private TimeOfFlight intakeSensor;

    private static Superstructure instance = new Superstructure();
    public static Superstructure getInstance() {
        return instance;
    }

    private Superstructure() {
        indexBeltAbove = new TalonSRX(SUPERSTRUCTURE_ABOVE_BELT);
        indexBeltBelow = new TalonSRX(SUPERSTRUCTURE_BELOW_BELT);
        deliveryBelt = new TalonSRX(SUPERSTRUCTURE_DELIVERY_BELT);

        extensionArm = new DoubleSolenoid(TRANS_LOW_ID, TRANS_HIGH_ID);
        ballsIntake = new TalonSRX(SUPERSTRUCTURE_INTAKE);

        deliverySensor = new TimeOfFlight(FLIGHT_SENSOR_DELIVERY);
        indexSensor = new TimeOfFlight(FLIGHT_SENSOR_INDEX);
        intakeSensor = new TimeOfFlight(FLIGHT_SENSOR_INTAKE);

        reset();
    }

    @Override public synchronized void readPeriodicInputs() {
        periodic.deliveryDistance = deliverySensor.getRange();
        periodic.indexDistance = indexSensor.getRange();
        periodic.intakeDistance = intakeSensor.getRange();
    }

    @Override public synchronized void writePeriodicOutputs() {
        indexBeltBelow.set(ControlMode.PercentOutput, periodic.indexBeltBottomDemand);
        indexBeltAbove.set(ControlMode.PercentOutput, periodic.indexBeltTopDemand);
        deliveryBelt.set(ControlMode.PercentOutput, periodic.deliveryBeltDemand);

        extensionArm.set(periodic.armExtension);
        ballsIntake.set(ControlMode.PercentOutput, periodic.intakeDemand);
    }

    @Override public void outputTelemetry() {

    }

    @Override public void reset() {
        periodic = new SuperIO();

        deliverySensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        indexSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
        intakeSensor.setRangingMode(TimeOfFlight.RangingMode.Short, 10);
    }

    // Setters
    public void setArmExtension(DoubleSolenoid.Value armExtension) {
        periodic.armExtension = armExtension;
    }

    public void setDeliveryBeltDemand(double deliveryBeltDemand) {
        periodic.deliveryBeltDemand = deliveryBeltDemand;
    }

    public void setIndexBeltsDemand(double indexBeltDemand) {
        periodic.indexBeltTopDemand = indexBeltDemand;
        periodic.indexBeltBottomDemand = indexBeltDemand;
    }

    public void setIndexBeltTopDemand(double indexBeltDemand) {
        periodic.indexBeltTopDemand = indexBeltDemand;
    }

    public void setIntakeDemand(double intakeDemand) {
        periodic.intakeDemand = intakeDemand;
    }

    // Getters
    public double getDeliveryDistance() {
        return periodic.deliveryDistance;
    }

    public double getIndexDistance() {
        return periodic.indexDistance;
    }

    public double getIntakeDistance() {
        return periodic.intakeDistance;
    }

    public class SuperIO extends Subsystem.PeriodicIO {
        // Indexer Data
        public double indexBeltTopDemand;
        public double indexBeltBottomDemand;
        public double deliveryBeltDemand;
        // Input Data
        public double intakeDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kOff;

        // Sensor Data
        private double deliveryDistance;
        private double indexDistance;
        private double intakeDistance;
    }
}
