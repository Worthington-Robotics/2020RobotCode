package frc.robot.actions.superaction;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Superstructure;

import static frc.robot.Constants.*;

public class ShootAction extends Action {
    private Superstructure superstructure;
    private ShootType shootType;

    public ShootAction(ShootType shootType) {
        superstructure = Superstructure.getInstance();

        this.shootType = shootType;
    }

    @Override public void onStart() {
        double demand =
                shootType == ShootType.ALL ? FULL_BELT_DEMAND : HIGH_BELT_DEMAND;

        superstructure.setDeliveryBeltDemand(HIGH_BELT_DEMAND);

        switch (shootType) {
            case ONE:
                superstructure.setIndexBeltTopDemand(HIGH_BELT_DEMAND);
                break;
            case ALL:
                superstructure.setIndexBeltsDemand(HIGH_BELT_DEMAND);
                break;
            default:
        }
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        return shootType == ShootType.ALL
                ? (Constants.DISTANCE_STOP_MM <= superstructure.getIndexDistance()) :
                (Constants.DISTANCE_STOP_MM >= superstructure.getDeliveryDistance());
    }

    @Override public void onStop() {
        superstructure.setDeliveryBeltDemand(STOP_BELT_DEMAND);
        superstructure.setIndexBeltsDemand(STOP_BELT_DEMAND);
    }
}
