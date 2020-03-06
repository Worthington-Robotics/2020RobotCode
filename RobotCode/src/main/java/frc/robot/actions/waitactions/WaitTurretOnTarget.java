package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class WaitTurretOnTarget extends Action {
    private Shooter shooter;

    /**
     * Code to run on action start.
     */
    @Override public void onStart() {
        shooter = Shooter.getInstance();
    }

    /**
     * Code to run while action loops.
     * <p>approx. every 20ms
     */
    @Override public void onLoop() {

    }

    /**
     * Method that tells the state machine the action is finished earlier than the scheduler.
     *
     * @return true action is ready to self terminate
     */
    @Override public boolean isFinished() {
        return shooter.onTarget();
    }

    /**
     * Code to run when the action has been called by the state machine to stop.
     */
    @Override public void onStop() {

    }
}
