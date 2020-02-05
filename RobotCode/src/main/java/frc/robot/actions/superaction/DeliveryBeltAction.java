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
        superstructure.setDeliveryBeltsDemand(Constants.HIGH_BELT_DEMAND);
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return Constants.DISTANCE_STOP_MM >= superstructure.getDeliveryDistance();
    }

    @Override public void onStop() {
        superstructure.setDeliveryBeltsDemand(Constants.STOP_BELT_DEMAND);
    }
}
