package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.lib.drivers.SimTimeOfFlight;
import frc.lib.util.TimerBoolean;

import static frc.robot.Constants.TIME_TILL_STATIONARY;

public class SuperstructureNew extends Subsystem {
    // Global
    private Superstructure.SuperIO periodic;

    // Indexer
    private TalonSRX deliveryWheel;
    private TalonSRX deliveryBelts;
    private TalonSRX indexTopBelt;
    private TalonSRX indexBottomBelt;

    // Intake
    private TalonSRX intakeWheels;
    private DoubleSolenoid extensionArm;

    // Sensors
    private SimTimeOfFlight deliverySensor;
    private SimTimeOfFlight indexSensor;
    private SimTimeOfFlight intakeSensor;

    // Double
    private double distanceDelivery;
    private double distanceIndexer;
    private double distanceIntake;


    private static SuperstructureNew instance = new SuperstructureNew();

    public static SuperstructureNew getInstance() {
        return instance;
    }

    /**
     * Updates all periodic variables and sensors
     */
    @Override public void readPeriodicInputs() {

    }

    /**
     * Writes the periodic outputs to actuators (motors and ect...)
     */
    @Override public void writePeriodicOutputs() {

    }

    /**
     * Outputs all logging information to the SmartDashboard
     */
    @Override public void outputTelemetry() {

    }

    /**
     * Called to reset and configure the subsystem
     */
    @Override public void reset() {

    }
}
