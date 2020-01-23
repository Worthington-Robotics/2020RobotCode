package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Intake extends Subsystem {
    private DoubleSolenoid solenoid;
    private TalonSRX srx;

    private static Intake instance = new Intake();
    public static Intake getInstance() {
        return instance;
    }

    @Override public void readPeriodicInputs() {

    }

    @Override public void writePeriodicOutputs() {

    }

    @Override public void outputTelemetry() {

    }

    @Override public void reset() {

    }
}
