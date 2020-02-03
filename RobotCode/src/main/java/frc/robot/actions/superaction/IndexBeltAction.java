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
        superstructure.setIndexBeltsDemand(Constants.HIGH_BELT_DEMAND);
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return Constants.DISTANCE_STOP_MM >= superstructure.getIndexDistance();
    }

    @Override public void onStop() {
        superstructure.setIndexBeltsDemand(Constants.STOP_BELT_DEMAND);

    }
}
