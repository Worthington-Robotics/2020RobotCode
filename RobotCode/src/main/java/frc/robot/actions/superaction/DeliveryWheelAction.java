package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

public class DeliveryWheelAction extends Action {
    private Superstructure superstructure;

    public DeliveryWheelAction() {
        superstructure = Superstructure.getInstance();
    }

    @Override public void onStart() {
        superstructure.setDeliveryWheelDemand(-Constants.HIGH_BELT_DEMAND);
    }

    @Override public void onLoop() {}

    @Override public boolean isFinished() {
        return false;
    }

    @Override public void onStop() {
        superstructure.setDeliveryWheelDemand(Constants.SUPER_DEMAND_STOP);
    }
}
