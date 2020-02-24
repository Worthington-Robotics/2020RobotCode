package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class IndexBeltAction extends Action {
    private Superstructure superstructure;
    private boolean isReversed;

    public IndexBeltAction(boolean isReversed) {
        superstructure = Superstructure.getInstance();
        this.isReversed = isReversed;
    }

    @Override public void onStart() {
        
        if(isReversed)
        superstructure.setIndexBeltDemand(-Constants.INDEXER_DEMAND);
        else
        superstructure.setIndexBeltDemand(Constants.INDEXER_DEMAND);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setIndexBeltDemand(Constants.STOP_DEMAND);

    }
}
