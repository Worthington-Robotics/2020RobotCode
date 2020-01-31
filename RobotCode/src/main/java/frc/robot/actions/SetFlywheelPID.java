package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class SetFlywheelPID extends Action {

    /**
     * code to run on action start
     * 
     */
    @Override
    public void onStart() {
        double length = Shooter.getInstance().limelightRanging();
        double RPM = Shooter.getInstance().calculateRPM(length);
        double speed = Shooter.getInstance().RPMToTicksPer100ms(RPM);
        Shooter.getInstance().setFlywheelRPM(speed);
    }

    /**
     * code to run while action loops
     * <p>approx every 20 miliseconds
     */
    @Override
        public void onLoop() {
        
    }

    /**
     * method that tells the state machine the action is finished earlier than the scheduler
     *
     * @return true when action is ready to self terminate
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    @Override
    public void onStop() {
    }
}
