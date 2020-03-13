package frc.robot.actions.waitactions;

import frc.lib.statemachine.Action;
import frc.robot.subsystems.PoseEstimator;

public class LineCrossWait extends Action {
    private boolean isY, end;
    private double lineLocation;
    private boolean backwards;

    public LineCrossWait(double coord, boolean isY, boolean isBackwards) {
        this.lineLocation = coord;
        this.isY = isY;
        backwards = isBackwards;
        end = false;
    }

    public void onStart() {

    }

    public void onLoop() {
        double currentVal = isY ? getY() : getX();

        if (backwards ? lineLocation >= currentVal : lineLocation <= currentVal) {
            end = true;
        }
    }

    private double getX() {
        return PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x();
    }

    private double getY() {
        return PoseEstimator.getInstance().getLatestFieldToVehicle().getValue().getTranslation().y();
    }

    public boolean isFinished() {
        return end;
    }

    public void onStop() {

    }
}
