package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class DeliveryBeltAction extends Action {
    private Superstructure superstructure;

    public DeliveryBeltAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setDeliveryBeltDemand(Constants.FULL_SPEED_BELT);
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setDeliveryBeltDemand(Constants.STOP_BELT);
    }
}
