package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Indexer extends Subsystem  {
    private DoubleSolenoid solenoid;
    private TalonSRX srx1, srx2, srx3;

    private static Indexer instance = new Indexer();
    public static Indexer getInstance() {
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
