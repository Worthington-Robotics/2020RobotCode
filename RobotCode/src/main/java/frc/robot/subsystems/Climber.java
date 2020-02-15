package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.Util;
import frc.robot.Constants;

public class Climber extends Subsystem {
    public DoubleSolenoid unfoldSolenoid, extendSolenoid;
    public DoubleSolenoid.Value unfoldCurrentState = Value.kReverse;
    public DoubleSolenoid.Value unfoldIntendedState = Value.kReverse;
    public DoubleSolenoid.Value extendCurrentState = Value.kReverse;
    public DoubleSolenoid.Value extendIntendedState = Value.kReverse;
    public boolean unfolded = false;
    public boolean climbed = false;
    public boolean intakeDown = false;
    public double shooterAngle = 90;

    public Climber() {
        unfoldSolenoid = new DoubleSolenoid(Constants.UNFOLD_LOW_ID, Constants.UNFOLD_HIGH_ID);
        extendSolenoid = new DoubleSolenoid(Constants.CLIMB_LOW_ID, Constants.CLIMB_HIGH_ID);
    }
    public static Climber mClimber = new Climber();
    public static Climber getInstance() {return mClimber;}
    
    @Override
    public void readPeriodicInputs() {
        unfoldCurrentState = unfoldSolenoid.get();
        extendCurrentState = extendSolenoid.get();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

			@Override
			public void onStart(double timestamp) {
				// TODO Auto-generated method stub
				
			}

			@Override
			public void onLoop(double timestamp) {
				if (!intakeDown) {
                    if ((Util.epsilonEquals(shooterAngle, 90, Constants.CLIMBER_EPSILON_CONST)) || (Util.epsilonEquals(shooterAngle, 90, Constants.CLIMBER_EPSILON_CONST))) {
                        if (unfoldCurrentState != unfoldIntendedState && extendCurrentState != Value.kForward) {
                            unfoldSolenoid.set(unfoldIntendedState);
                            //System.out.println("Unfold Happened");
                        }
                        if (unfoldCurrentState == Value.kForward) {
                            unfolded = true;
                        } else if (unfoldCurrentState == Value.kReverse) {
                           unfolded = false;
                        }
                    }
                }
                if (!unfolded) {
                    extendSolenoid.set(Value.kReverse);
                } else {
                    extendSolenoid.set(extendIntendedState);
                    //System.out.println("Extend Happened");
                }
                if (extendCurrentState == Value.kForward) {
                    climbed = true;
                } else if (extendCurrentState == Value.kReverse) {
                    climbed = false;
                }
				
			}

			@Override
			public void onStop(double timestamp) {
				// TODO Auto-generated method stub
				
			}

        });
    }

    @Override
    public void writePeriodicOutputs() {
        extendSolenoid.set(extendIntendedState);
        unfoldSolenoid.set(unfoldIntendedState);
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void reset() {

    }

    public void setUnfold(Value unfoldValue) {
        if (extendCurrentState != Value.kForward) {
            unfoldIntendedState = unfoldValue;
        }
    }

    public void setExtend(Value extendValue) {
        if (extendValue != extendIntendedState && unfolded) {
            extendIntendedState = extendValue;
        }
    }
    
}

