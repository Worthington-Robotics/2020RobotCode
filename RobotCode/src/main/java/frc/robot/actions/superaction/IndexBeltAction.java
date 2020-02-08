package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class IndexBeltAction extends Action {
    private Superstructure superstructure;

    public IndexBeltAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setIndexBeltDemand(Constants.HIGH_BELT_DEMAND);
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setIndexBeltDemand(Constants.STOP_BELT_DEMAND);

    }
}
