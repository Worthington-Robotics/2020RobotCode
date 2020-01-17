package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Climber extends Subsystem {
    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public DoubleSolenoid unfoldsolenoid, extendsolenoid;
    public Climber(){
            unfoldsolenoid = new DoubleSolenoid(2,3);
            extendsolenoid = new DoubleSolenoid(4,5);
        }


    }
    public class periodicio extends PeriodicIO {}
}
