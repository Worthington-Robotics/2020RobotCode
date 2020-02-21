package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;

public class AutoShootAction extends Action {
    private Shooter shooter;
    private Superstructure superstructure;

    /**
     * Code to run on action start.
     */
    @Override public void onStart() {
        shooter = Shooter.getInstance();
        superstructure = Superstructure.getInstance();
    }

    /**
     * Code to run while action loops.
     * <p>approx. every 20ms
     */
    @Override public void onLoop() {
        if (shooter.isTurretOnTarget()) {
            superstructure.shootBall();
        }
    }

    /**
     * Method that tells the state machine the action is finished earlier than the scheduler.
     *
     * @return true action is ready to self terminate
     */
    @Override public boolean isFinished() {
        return false;
    }

    /**
     * Code to run when the action has been called by the state machine to stop.
     */
    @Override public void onStop() {

    }
}
