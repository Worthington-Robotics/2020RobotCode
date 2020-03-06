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

    private Value unfoldIntendedState = Value.kReverse, climbIntendedState = Value.kReverse;
    private boolean wantUnfold = false, wantClimb = false;
    private boolean climbBoolean, unfoldBoolean;

    private Climber() {
        unfoldSolenoid = new DoubleSolenoid(Constants.UNFOLD_LOW_ID, Constants.UNFOLD_HIGH_ID);
        climbSolenoid = new DoubleSolenoid(Constants.CLIMB_LOW_ID, Constants.CLIMB_HIGH_ID);
        reset();

        //SmartDashboard.putNumber("turretsim", 0);
    }

    private static Climber mClimber = new Climber();

    public static Climber getInstance() {
        return mClimber;
    }

    @Override
    public void readPeriodicInputs() {
        
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {

            @Override
            public void onStart(double timestamp) {
                // TODO Auto-generated method stub

            }

            @Override
            public void onLoop(double timestamp) {
                if(!Constants.DEBUG){
                    if(canUnfold()){
                        unfoldBoolean = wantUnfold;
                    }
                
                    if(canClimb()){
                        climbBoolean = wantClimb;
                    }
                    
                } else {
                    unfoldBoolean = wantUnfold;
                    climbBoolean = wantClimb;
                }

                unfoldIntendedState = unfoldBoolean ? Value.kForward : Value.kReverse;
                climbIntendedState = climbBoolean ? Value.kForward : Value.kReverse;                
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
        unfoldSolenoid.set(unfoldIntendedState);
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Climb/Want Unfolded", wantUnfold);
        SmartDashboard.putBoolean("Climb/Want Extended", wantClimb);
        SmartDashboard.putBoolean("Climb/Unfolded", unfoldBoolean);
        SmartDashboard.putBoolean("Climb/Climbed", climbBoolean);
        SmartDashboard.putBoolean("Climb/ShooterReady", shooterReady());
    }

    public boolean canUnfold(){
        return shooterReady();
    }

    public boolean canClimb(){
        return unfoldBoolean;
    }

    /*private double getTurretAngle(){
        return SmartDashboard.getNumber("turretsim", 0);
    }*/

    private boolean shooterReady(){
        return Util.epsilonEquals(Shooter.getInstance().getShooterAngle(), 270, 25) ||
        Util.epsilonEquals(Shooter.getInstance().getShooterAngle(), 90, 25);

        /*return Util.epsilonEquals(getTurretAngle(), 270, 10) ||
        Util.epsilonEquals(getTurretAngle(), 90, 10);*/
    }

    @Override
    public void reset() {
        wantUnfold = wantClimb = false;
        unfoldBoolean = climbBoolean = false;
        unfoldSolenoid.set(Value.kReverse);
        climbSolenoid.set(Value.kReverse);
    }

    public void wantUnfold(Boolean unfoldValue) {
        wantUnfold = unfoldValue;
    }

    public void wantClimb(Boolean extendValue) {
        wantClimb = extendValue;
    }

    public boolean getUnfolded() {
        return unfoldBoolean;
    }

    public boolean getClimbed() {
        return climbBoolean;
    }

}
