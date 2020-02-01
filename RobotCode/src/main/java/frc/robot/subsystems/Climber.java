package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber extends Subsystem {
    public DoubleSolenoid unfoldSolenoid, extendSolenoid;
    public DoubleSolenoid.Value unfoldCurrentState, unfoldIntendedState, extendCurrentState, extendIntendedState;
    public boolean unfolded = false;

    public Climber() {
        unfoldSolenoid = new DoubleSolenoid(2, 3);
        extendSolenoid = new DoubleSolenoid(4, 5);
    }
    public Climber mClimber = new Climber();
    public Climber getInstance() {return mClimber;}

    
    @Override
    public void readPeriodicInputs() {
        unfoldCurrentState = unfoldSolenoid.get();
        extendCurrentState = extendSolenoid.get();
    }

    @Override
    public void writePeriodicOutputs() {
        if (unfoldCurrentState != unfoldIntendedState) {
            unfoldSolenoid.set(unfoldIntendedState);
        }
        if (unfoldCurrentState == Value.kForward) {
            unfolded = true;
        } else if (unfoldCurrentState == Value.kReverse) {
            unfolded = false;
        }
        if (extendCurrentState != extendIntendedState && unfoldCurrentState == Value.kForward) {
            extendSolenoid.set(extendIntendedState);
        }
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public void setUnfold(Value unfoldValue) {
        unfoldIntendedState = unfoldValue;
    }

    public void setExtend(Value extendValue) {
        extendIntendedState = extendValue;
    }
    
}

