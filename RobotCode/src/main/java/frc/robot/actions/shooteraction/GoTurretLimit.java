package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class GoTurretLimit extends Action {
    private Shooter shooter;
    private boolean negative;

    public GoTurretLimit(boolean negative) {
        shooter = Shooter.getInstance();
        this.negative = negative;
    }

    /**
     * Code to run on action start.
     */
    @Override public void onStart() {
        shooter.setTurretDemand(Constants.TURRET_MAX_SPEED);
    }

    /**
     * Code to run while action loops.
     * <p>approx. every 20ms
     */
    @Override public void onLoop() {}

    /**
     * Tells the state machine the action is finished earlier than the scheduler.
     *
     * @return true action is ready to self terminate
     */
    @Override public boolean isFinished() {
        return negative ?
                shooter.getTurretEncoder() <= Constants.LEFT_TURRET_LIMIT :
                shooter.getTurretEncoder() >= Constants.RIGHT_TURRET_LIMIT;
    }

    /**
     * Code to run when the action has been called by the state machine to stop.
     */
    @Override public void onStop() {
        //shooter.setTurretDemand(Constants.STOP_DEMAND);
    }
}
