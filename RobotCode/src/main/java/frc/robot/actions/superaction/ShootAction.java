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
        if (superstructure.getBallCount() > 0) {
            double demand =
                (shootType == ShootType.ALL ? FULL_BELT_DEMAND : HIGH_BELT_DEMAND);

            superstructure.setDeliveryBeltsDemand(demand);
            superstructure.setIndexBeltDemand(demand);
        }
    }

    @Override public void onLoop() {

    }

    @Override public boolean isFinished() {
        boolean ballsShot = superstructure.getBallCount() > 0 && (ShootType.ALL == shootType
            ? (Constants.DISTANCE_EMPTY_MM <= superstructure.getIndexDistance()) :
            (Constants.DISTANCE_STOP_MM >= superstructure.getDeliveryDistance()));

        if (ballsShot) {
            superstructure.setBallCount(ShootType.ALL == shootType ? 0 : (superstructure.getBallCount() - 1));
        }

        return shootType == ShootType.ALL
                ? (Constants.DISTANCE_STOP_MM <= superstructure.getIndexDistance()) :
                (Constants.DISTANCE_STOP_MM >= superstructure.getDeliveryDistance());
    }  

    @Override public void onStop() {
        superstructure.setDeliveryBeltsDemand(STOP_BELT_DEMAND);
        superstructure.setIndexBeltDemand(STOP_BELT_DEMAND);
    }
}
