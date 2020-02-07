package frc.robot.actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class ManualTurretControl extends Action {
        public boolean isCounterClockwise;
    public ManualTurretControl(boolean CCW)
    {
        isCounterClockwise = CCW;
        System.out.println("Activated");
    }
    /**
     * code to run on action start
     */
    @Override
    public void onStart() {
        if(isCounterClockwise)
        {
            Shooter.getInstance().setTurretDemand(Constants.TURRET_MAX_SPEED);
        }else{
            Shooter.getInstance().setTurretDemand(-1 * Constants.TURRET_MAX_SPEED);
        }
        System.out.println("Activated");
    }

    /**
     * code to run while action loops
     * <p>approx every 20 miliseconds
     */
    @Override
    public void onLoop() {
        System.out.print("Done");

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
        Shooter.getInstance().setTurretDemand(0);
    }
}
