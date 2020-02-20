package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
//import frc.robot.subsystems.PoseEstimator;

public class WaitLineCross extends Action {
    private boolean isX, end;
    private double mCoord;

    public WaitLineCross(double coord, boolean isX) {
        mCoord = coord;
        this.isX = isX;
        end = false;
    }

    public void onStart() {

    }

    public void onLoop() {
        /*if (isX) {
            if (mCoord < PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x()) {
                end = true;
            }
        } else {
            if (mCoord < PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y()) {
                end = true;
            }
        }*/
    }

    public boolean isFinished() {
        return end;
    }

    public void onStop() {

    }
}
