package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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

        reset();
    }

    @Override public synchronized void readPeriodicInputs() {

    }

    @Override public synchronized void writePeriodicOutputs() {
        indexBeltBelow.set(ControlMode.PercentOutput, periodic.indexBeltDemand);
        indexBeltAbove.set(ControlMode.PercentOutput, periodic.indexBeltDemand);
        deliveryBelt.set(ControlMode.PercentOutput, periodic.deliveryBeltDemand);

        extensionArm.set(periodic.armExtension);
        ballsIntake.set(ControlMode.PercentOutput, periodic.intakeDemand);
    }

    @Override public void outputTelemetry() {

    }

    @Override public void reset() {
        periodic = new SuperIO();
    }

    public void setIndexBeltDemand(double indexBeltDemand) {
        periodic.indexBeltDemand = indexBeltDemand;
    }

    public void setDeliveryBeltDemand(double deliveryBeltDemand) {
        periodic.deliveryBeltDemand = deliveryBeltDemand;
    }

    public void setIntakeDemand(double intakeDemand) {
        periodic.intakeDemand = intakeDemand;
    }

    public void setArmExtension(DoubleSolenoid.Value armExtension) {
        periodic.armExtension = armExtension;
    }

    public class SuperIO extends Subsystem.PeriodicIO {
        public double indexBeltDemand;
        public double deliveryBeltDemand;

        public double intakeDemand;
        public DoubleSolenoid.Value armExtension = DoubleSolenoid.Value.kOff;
    }
}
