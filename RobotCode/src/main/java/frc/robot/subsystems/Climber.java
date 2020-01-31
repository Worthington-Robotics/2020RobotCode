package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber extends Subsystem {
    public DoubleSolenoid unfoldSolenoid, extendSolenoid;
    public boolean unfoldCurrentState, unfoldIntendedState, extendCurrentState, extendIntendedState;
    public boolean climberUp;

    public Climber() {
        unfoldSolenoid = new DoubleSolenoid(2, 3);
        extendSolenoid = new DoubleSolenoid(4, 5);
    }
    public Climber mClimber = new Climber();
    public Climber getInstance() {return mClimber;}

    
    @Override
    public void readPeriodicInputs() {

        if (unfoldSolenoid.get() == Value.kForward) {
            unfoldCurrentState = true;
        } else if (unfoldSolenoid.get() == Value.kReverse) {
            unfoldCurrentState = false;
        }
        if (extendSolenoid.get() == Value.kForward) {
            extendCurrentState = true;
        } else if (extendSolenoid.get() == Value.kReverse) {
            extendCurrentState = false;
        }
    }

    @Override
    public void writePeriodicOutputs() {
        if (unfoldCurrentState != unfoldIntendedState) {

        }
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public void setUnfold(Value open) {

    }

    public class periodicIO extends Subsystem.PeriodicIO {
        public DoubleSolenoid.Value unfoldsolenoidinput = DoubleSolenoid.Value.kOff;
        public DoubleSolenoid.Value extendsolenoidinput = DoubleSolenoid.Value.kOff;
    }
}
