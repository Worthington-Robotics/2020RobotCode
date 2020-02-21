package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.loops.ILooper;
import frc.lib.loops.Loop;
import frc.lib.util.Util;
import frc.robot.Constants;

public class Climber extends Subsystem {
    private DoubleSolenoid unfoldSolenoid, climbSolenoid;
    private Value unfoldCurrentState = Value.kReverse, unfoldIntendedState = Value.kReverse,
            climbCurrentState = Value.kReverse, climbIntendedState = Value.kReverse;
            private boolean unfolded = false, climbed = false, intakeDown = false;
            private double shooterAngle = 90;
            private boolean climbBoolean, unfoldBoolean, readyToDownClimb, readyToFold, readyToUnfold, readyToClimb,
            readyFolding;

            private Climber() {
        unfoldSolenoid = new DoubleSolenoid(Constants.UNFOLD_LOW_ID, Constants.UNFOLD_HIGH_ID);
        climbSolenoid = new DoubleSolenoid(Constants.CLIMB_LOW_ID, Constants.CLIMB_HIGH_ID);
        reset();
    }

    private static Climber mClimber = new Climber();

    public static Climber getInstance() {
        return mClimber;
    }

    @Override
    public void readPeriodicInputs() {
        unfoldCurrentState = unfoldSolenoid.get();
        climbCurrentState = climbSolenoid.get();
        intakeDown = true;
        // shooterAngle = Shooter.getInstance();
        // intakeDown = Superstructure.getInstance().getIntakeDown();
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onLoop(double timestamp) {
                if (!Constants.DEBUG) {
                    if (intakeDown) {
                        if ((Util.epsilonEquals(shooterAngle, Constants.CLIMBER_SHOOTER_REQMT,
                                Constants.CLIMBER_EPSILON_CONST))) {
                            unfoldIntendedState = unfoldBoolean ? Value.kForward : Value.kReverse;
                            // System.out.println("Unfold Happened");
                        }
                    }
                    unfolded = unfoldCurrentState == Value.kForward;

                    if (unfolded) {
                        climbIntendedState = climbBoolean ? Value.kForward : Value.kReverse;
                    } else {
                        climbIntendedState = Value.kReverse;
                        climbBoolean = false;
                    }

                    climbed = climbCurrentState == Value.kForward;

                } else {
                    unfoldIntendedState = unfoldBoolean ? Value.kForward : Value.kReverse;
                    climbIntendedState = climbBoolean ? Value.kForward : Value.kReverse;
                }
                readyToClimb = unfolded;
                readyToDownClimb = unfolded && climbed;
                readyToUnfold = !climbed;
                readyToFold = unfolded && !climbed;
            }

            @Override
            public void onStop(double timestamp) {
                // TODO Auto-generated method stub

            }

        });
    }

    @Override
    public void writePeriodicOutputs() {
        climbSolenoid.set(climbIntendedState);
        if (Shooter.getInstance().canUnfold())
            unfoldSolenoid.set(unfoldIntendedState);
    }

    public boolean readyToFold(){
        return readyToFold;
    }

    public boolean readyToUnfold(){
        return readyToUnfold;
    }

    public boolean readyToClimb(){
        return readyToClimb;
    }

    public boolean readyToDownClimb(){
        return readyToDownClimb;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Climb/Want Unfolded", unfoldBoolean);
        SmartDashboard.putBoolean("Climb/Want Extended", climbBoolean);
        SmartDashboard.putBoolean("Climb/Unfolded", unfolded);
        SmartDashboard.putBoolean("Climb/Extend", climbed);
    }

    @Override
    public void reset() {
        unfoldSolenoid.set(Value.kReverse);
        climbSolenoid.set(Value.kForward);
    }

    public void setUnfold(Boolean unfoldValue) {
        unfoldBoolean = unfoldValue;
    }

    public void setExtend(Boolean extendValue) {
        climbBoolean = extendValue;
    }

    public boolean getUnfolded() {
        return unfolded;
    }

    public boolean getClimbed() {
        return climbed;
    }

}
