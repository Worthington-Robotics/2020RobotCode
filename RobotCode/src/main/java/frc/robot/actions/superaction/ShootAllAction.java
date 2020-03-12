package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.lib.util.TimerBoolean;
import frc.robot.subsystems.Superstructure;

public class ShootAllAction extends Action {
    private TimerBoolean emptyWait = new TimerBoolean(.5);
    private boolean finishWhenEmpty;

    public ShootAllAction() {}

    public ShootAllAction(boolean finishWhenEmpty) {
        this.finishWhenEmpty = finishWhenEmpty;
    }

    @Override public void onStart() {}

    @Override public void onLoop() {
        Superstructure.getInstance().shootAll();

        if (finishWhenEmpty) {
            if (Superstructure.getInstance().isSystemEmpty()) {
                emptyWait.start();
            } else if (emptyWait.isStarted()) {
                emptyWait.stop();
            }
        }
    }

    @Override public boolean isFinished() {
        return finishWhenEmpty ? emptyWait.getBoolean() : false;
    }

    @Override public void onStop() {
        Superstructure.getInstance().setInit();
    }
}
