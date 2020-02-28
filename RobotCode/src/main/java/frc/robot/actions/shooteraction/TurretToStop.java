package frc.robot.actions.shooteraction;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.Shooter;

public class TurretToStop extends Action {
    public boolean isReversed = false;
    public TurretToStop(boolean isReversed)
    {
        this.isReversed = isReversed;
    }

    /**
     * code to run on action start
     * 
     */
    @Override
    public void onStart() {
        Shooter.getInstance().setTurretDemand(.2);
    }

    /**
     * code to run while action loops
     * <p>
     * approx every 20 miliseconds
     */
    @Override
    public void onLoop() {
        Shooter.getInstance().setTurretDemand(.2);
    }

    /**
     * method that tells the state machine the action is finished earlier than the
     * scheduler
     *
     * @return true when action is ready to self terminate
     */
    @Override
    public boolean isFinished() {
        return Shooter.getInstance().getSupplyCurrent() > 4;
    }

    /**
     * code to run when the action has ben called by the state machine to stop
     */
    @Override
    public void onStop() {
        Shooter.getInstance().setTurretDemand(0);
    }
}
